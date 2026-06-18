#!/usr/bin/env python3
import json
import math
import os
import signal
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


LEVEL_RANK = {
    'unknown': 0,
    'clear': 1,
    'warning': 2,
    'critical': 3,
}


class SysfsOutput:
    def __init__(self, gpio_base, bcm_gpio, active_high=True):
        self.gpio = int(gpio_base) + int(bcm_gpio)
        self.active_high = bool(active_high)
        self.path = f'/sys/class/gpio/gpio{self.gpio}'
        self.value_fd = None
        if not os.path.isdir(self.path):
            with open('/sys/class/gpio/export', 'w', encoding='utf-8') as handle:
                handle.write(str(self.gpio))
            time.sleep(0.05)
        with open(os.path.join(self.path, 'direction'), 'w', encoding='utf-8') as handle:
            handle.write('out')
        self.value_fd = os.open(os.path.join(self.path, 'value'), os.O_WRONLY)
        self.write(False)

    def write(self, on):
        value = bool(on)
        if not self.active_high:
            value = not value
        os.lseek(self.value_fd, 0, os.SEEK_SET)
        os.write(self.value_fd, b'1' if value else b'0')

    def tone(self, frequency_hz, duration_sec):
        frequency = max(100.0, float(frequency_hz))
        duration = max(0.0, float(duration_sec))
        half_period = 0.5 / frequency
        deadline = time.perf_counter() + duration
        old_switch_interval = sys.getswitchinterval()
        sys.setswitchinterval(0.05)
        try:
            while time.perf_counter() < deadline:
                self.write(True)
                self._wait_half_period(half_period)
                self.write(False)
                self._wait_half_period(half_period)
        finally:
            self.write(False)
            sys.setswitchinterval(old_switch_interval)

    @staticmethod
    def _wait_half_period(seconds):
        deadline = time.perf_counter() + seconds
        while time.perf_counter() < deadline:
            pass


class PwmOutput:
    def __init__(self, pwm_chip, pwm_channel, duty_ratio=0.5):
        self.path = os.path.join(str(pwm_chip), f'pwm{int(pwm_channel)}')
        self.duty_ratio = max(0.05, min(0.95, float(duty_ratio)))
        if not os.path.isdir(self.path):
            export_path = os.path.join(str(pwm_chip), 'export')
            with open(export_path, 'w', encoding='utf-8') as handle:
                handle.write(str(int(pwm_channel)))
            time.sleep(0.1)
        if not os.path.isdir(self.path):
            raise FileNotFoundError(self.path)
        self._write('enable', 0)

    def _write(self, name, value):
        with open(os.path.join(self.path, name), 'w', encoding='utf-8') as handle:
            handle.write(str(value))

    def write(self, on):
        self._write('enable', 1 if on else 0)

    def tone(self, frequency_hz, duration_sec):
        frequency = max(50.0, float(frequency_hz))
        duration = max(0.0, float(duration_sec))
        period_ns = max(1, int(1_000_000_000 / frequency))
        duty_ns = max(1, int(period_ns * self.duty_ratio))
        self._write('enable', 0)
        self._write('duty_cycle', 0)
        self._write('period', period_ns)
        self._write('duty_cycle', duty_ns)
        self._write('enable', 1)
        time.sleep(duration)
        self._write('enable', 0)


class IotSupervisorNode(Node):
    """Unify machine safety signals, drive the buzzer, and report to the base station."""

    def __init__(self):
        super().__init__('iot_supervisor_node')

        self.declare_parameter('machine_id', 'machine-1')
        self.declare_parameter('fusion_topic', '/fusion_status')
        self.declare_parameter('uwb_topic', '/uwb/workers')
        self.declare_parameter('ultrasonic_topic', '/ultrasonic/status')
        self.declare_parameter('status_topic', '/iot/status')
        self.declare_parameter('alert_topic', '/iot/alert_state')
        self.declare_parameter('report_topic', '/iot/accident_reports')
        self.declare_parameter('base_station_url', 'http://10.152.83.65:8090')
        self.declare_parameter('viewer_base_url', 'http://10.152.83.176:8080')
        self.declare_parameter('video_clip_before_sec', 5.0)
        self.declare_parameter('video_clip_after_sec', 5.0)
        self.declare_parameter('video_clip_fps', 4.0)
        self.declare_parameter('evidence_before_sec', 5.0)
        self.declare_parameter('evidence_after_sec', 5.0)
        self.declare_parameter('uwb_history_period_sec', 0.25)
        self.declare_parameter('ping_period_sec', 5.0)
        self.declare_parameter('report_cooldown_sec', 60.0)
        self.declare_parameter('source_timeout_sec', 3.0)
        self.declare_parameter('buzzer_enabled', True)
        self.declare_parameter('buzzer_driver', 'gpio')
        self.declare_parameter('buzzer_gpio', 19)
        self.declare_parameter('gpio_base', 571)
        self.declare_parameter('buzzer_active_high', True)
        self.declare_parameter('pwm_chip', '/sys/class/pwm/pwmchip0')
        self.declare_parameter('pwm_channel', 1)
        self.declare_parameter('pwm_duty_ratio', 0.5)
        self.declare_parameter('warning_tone_hz', 1800.0)
        self.declare_parameter('critical_tone_hz', 2200.0)
        self.declare_parameter('warning_buzz_period_sec', 1.0)
        self.declare_parameter('critical_buzz_period_sec', 0.32)
        self.declare_parameter('buzz_duration_sec', 0.22)

        self.machine_id = self.get_parameter('machine_id').value
        self.base_station_url = str(
            self.get_parameter('base_station_url').value
        ).rstrip('/')
        self.viewer_base_url = str(
            self.get_parameter('viewer_base_url').value
        ).rstrip('/')
        self.video_clip_before = float(
            self.get_parameter('video_clip_before_sec').value
        )
        self.video_clip_after = float(
            self.get_parameter('video_clip_after_sec').value
        )
        self.video_clip_fps = float(self.get_parameter('video_clip_fps').value)
        self.evidence_before = float(
            self.get_parameter('evidence_before_sec').value
        )
        self.evidence_after = float(
            self.get_parameter('evidence_after_sec').value
        )
        self.uwb_history_period = float(
            self.get_parameter('uwb_history_period_sec').value
        )
        self.ping_period = float(self.get_parameter('ping_period_sec').value)
        self.report_cooldown = float(
            self.get_parameter('report_cooldown_sec').value
        )
        self.source_timeout = float(
            self.get_parameter('source_timeout_sec').value
        )

        self.status_pub = self.create_publisher(
            String,
            self.get_parameter('status_topic').value,
            10,
        )
        self.alert_pub = self.create_publisher(
            String,
            self.get_parameter('alert_topic').value,
            10,
        )
        self.report_pub = self.create_publisher(
            String,
            self.get_parameter('report_topic').value,
            10,
        )

        self.create_subscription(
            String,
            self.get_parameter('fusion_topic').value,
            self.fusion_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('uwb_topic').value,
            self.uwb_callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('ultrasonic_topic').value,
            self.ultrasonic_callback,
            10,
        )

        self.sources = {}
        self.uwb_history = deque()
        self.pending_reports = []
        self.last_uwb_history_time = 0.0
        self.last_ping_time = 0.0
        self.last_report_time = 0.0
        self.last_report_signature = None
        self.last_buzz_time = 0.0
        self.last_level = 'unknown'
        self.buzz_lock = threading.Lock()
        self.buzzer = self._make_buzzer()

        self.timer = self.create_timer(0.2, self.tick)
        self.get_logger().info(
            f'IoT supervisor active for {self.machine_id}; '
            f'base={self.base_station_url or "disabled"}'
        )

    def _make_buzzer(self):
        if not bool(self.get_parameter('buzzer_enabled').value):
            return None
        driver = str(self.get_parameter('buzzer_driver').value).lower()
        if driver in ('auto', 'pwm'):
            try:
                buzzer = PwmOutput(
                    pwm_chip=self.get_parameter('pwm_chip').value,
                    pwm_channel=self.get_parameter('pwm_channel').value,
                    duty_ratio=self.get_parameter('pwm_duty_ratio').value,
                )
                self.get_logger().info('Buzzer using kernel PWM')
                return buzzer
            except Exception as exc:
                if driver == 'pwm':
                    self.get_logger().warning(f'Buzzer disabled: {exc}')
                    return None
                self.get_logger().warning(f'PWM buzzer unavailable, falling back to GPIO: {exc}')
        try:
            buzzer = SysfsOutput(
                gpio_base=self.get_parameter('gpio_base').value,
                bcm_gpio=self.get_parameter('buzzer_gpio').value,
                active_high=self.get_parameter('buzzer_active_high').value,
            )
            self.get_logger().info('Buzzer using GPIO software tone')
            return buzzer
        except Exception as exc:
            self.get_logger().warning(f'Buzzer disabled: {exc}')
            return None

    def fusion_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        obstacle = payload.get('obstacle') or {}
        level = 'critical' if obstacle.get('critical') else 'clear'
        detail = {
            'distance_m': obstacle.get('distance_m'),
            'matched_person': obstacle.get('matched_person'),
            'raw': obstacle,
        }
        self._set_source('fusion', level, detail)

    def uwb_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        now = time.time()
        payload.setdefault('time', now)
        self._record_uwb_sample(payload, now)
        workers = payload.get('workers', [])
        level = 'clear'
        closest = None
        for worker in workers:
            worker_level = worker.get('warning_level', 'clear')
            if LEVEL_RANK.get(worker_level, 0) > LEVEL_RANK.get(level, 0):
                level = worker_level
            distance = worker.get('distance_to_machine_m')
            if distance is not None and (
                closest is None or distance < closest.get('distance_to_machine_m', math.inf)
            ):
                closest = worker
        detail = {
            'workers': workers,
            'closest_worker': closest,
            'raw': payload,
        }
        self._set_source('uwb', level, detail)

    def ultrasonic_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        level = payload.get('overall_level', 'unknown')
        self._set_source('ultrasonic', level, payload)

    def _set_source(self, name, level, detail):
        if level not in LEVEL_RANK:
            level = 'unknown'
        self.sources[name] = {
            'level': level,
            'detail': detail,
            'time': time.time(),
        }

    def tick(self):
        status = self._build_status()
        self._publish(self.status_pub, status)
        self._publish(self.alert_pub, {
            'time': status['time'],
            'machine_id': self.machine_id,
            'level': status['level'],
            'active_sources': status['active_sources'],
        })
        self._drive_buzzer(status['level'])
        self._maybe_send_ping(status)
        self._maybe_report(status)
        self._publish_due_reports()
        self.last_level = status['level']

    def _build_status(self):
        now = time.time()
        active_sources = {}
        level = 'clear'
        for name, source in self.sources.items():
            age = now - float(source.get('time', 0.0))
            if age > self.source_timeout:
                source_level = 'unknown'
            else:
                source_level = source.get('level', 'unknown')
            if LEVEL_RANK.get(source_level, 0) > LEVEL_RANK.get(level, 0):
                level = source_level
            active_sources[name] = {
                'level': source_level,
                'age_sec': age,
                'detail': source.get('detail', {}),
            }
        return {
            'type': 'machine_ping',
            'time': now,
            'device_id': self.machine_id,
            'device_type': 'machine',
            'level': level,
            'active_sources': active_sources,
            'media': self._build_media(now),
        }

    @staticmethod
    def _publish(publisher, payload):
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        publisher.publish(msg)

    def _drive_buzzer(self, level):
        if self.buzzer is None or level not in ('warning', 'critical'):
            return
        now = time.time()
        if level == 'critical':
            period = float(self.get_parameter('critical_buzz_period_sec').value)
            frequency = float(self.get_parameter('critical_tone_hz').value)
        else:
            period = float(self.get_parameter('warning_buzz_period_sec').value)
            frequency = float(self.get_parameter('warning_tone_hz').value)
        if now - self.last_buzz_time < period:
            return
        self.last_buzz_time = now
        duration = float(self.get_parameter('buzz_duration_sec').value)
        threading.Thread(
            target=self._buzz_once,
            args=(frequency, duration),
            daemon=True,
        ).start()

    def _buzz_once(self, frequency, duration):
        if not self.buzz_lock.acquire(blocking=False):
            return
        try:
            self.buzzer.tone(frequency, duration)
        except Exception as exc:
            self.get_logger().warning(f'Buzzer write failed: {exc}')
            self.buzzer = None
        finally:
            self.buzz_lock.release()

    def _maybe_send_ping(self, status):
        now = time.time()
        if now - self.last_ping_time < self.ping_period:
            return
        self.last_ping_time = now
        self._post_json('/api/ping', status)

    def _maybe_report(self, status):
        if status['level'] != 'critical':
            return
        now = float(status.get('time', time.time()))
        signature = self._critical_signature(status)
        should_report = (
            self.last_level != 'critical'
            or now - self.last_report_time >= self.report_cooldown
        )
        if not should_report:
            return
        self.last_report_time = now
        self.last_report_signature = signature
        self.pending_reports.append({
            'accident_time': now,
            'due_time': now + max(0.0, self.evidence_after),
            'signature': signature,
            'status': self._json_copy(status),
        })

    def _publish_due_reports(self):
        if not self.pending_reports:
            return
        now = time.time()
        remaining = []
        due = []
        for item in self.pending_reports:
            if now >= float(item.get('due_time', 0.0)):
                due.append(item)
            else:
                remaining.append(item)
        self.pending_reports = remaining
        for item in due:
            accident_time = float(item.get('accident_time', now))
            status = item.get('status') or {}
            signature = item.get('signature', '')
            report_id = self._make_report_id(accident_time, signature)
            media = self._build_recorded_media(accident_time, report_id)
            report = {
                'type': 'accident_report',
                'report_id': report_id,
                'time': accident_time,
                'generated_time': now,
                'device_id': self.machine_id,
                'device_type': 'machine',
                'level': 'critical',
                'signature': signature,
                'media': media,
                'evidence': self._build_evidence(accident_time, status),
                'status': status,
            }
            self._publish(self.report_pub, report)
            self._post_json('/api/report', report)

    @staticmethod
    def _json_copy(value):
        return json.loads(json.dumps(value))

    def _record_uwb_sample(self, payload, now):
        if now - self.last_uwb_history_time < max(0.05, self.uwb_history_period):
            return
        sample = self._json_copy(payload)
        sample['time'] = float(sample.get('time') or now)
        self.uwb_history.append(sample)
        self.last_uwb_history_time = now
        max_age = max(30.0, self.evidence_before + self.evidence_after + 10.0)
        cutoff = now - max_age
        while self.uwb_history and float(self.uwb_history[0].get('time', 0.0)) < cutoff:
            self.uwb_history.popleft()

    def _build_evidence(self, center_time, status):
        center = float(center_time)
        before = max(0.0, self.evidence_before)
        after = max(0.0, self.evidence_after)
        start = center - before
        end = center + after
        history = [
            self._json_copy(sample)
            for sample in list(self.uwb_history)
            if start <= float(sample.get('time', 0.0)) <= end
        ]
        snapshot = self._closest_uwb_sample(center, max(before, after, 1.0))
        if snapshot is None:
            snapshot = self._uwb_snapshot_from_status(status, center)
        if not history and snapshot is not None:
            history = [self._json_copy(snapshot)]
        return {
            'window': {
                'center_time': center,
                'start_time': start,
                'end_time': end,
                'before_sec': before,
                'after_sec': after,
            },
            'uwb_snapshot': snapshot or {},
            'uwb_history': history,
        }

    def _closest_uwb_sample(self, center_time, max_delta=None):
        closest = None
        best_delta = math.inf
        for sample in list(self.uwb_history):
            sample_time = float(sample.get('time', 0.0))
            delta = abs(sample_time - center_time)
            if delta < best_delta:
                best_delta = delta
                closest = sample
        if max_delta is not None and best_delta > float(max_delta):
            return None
        return self._json_copy(closest) if closest is not None else None

    def _uwb_snapshot_from_status(self, status, center_time):
        detail = (
            status.get('active_sources', {})
            .get('uwb', {})
            .get('detail', {})
        )
        raw = detail.get('raw') if isinstance(detail, dict) else None
        if not isinstance(raw, dict):
            return None
        snapshot = self._json_copy(raw)
        snapshot['time'] = float(snapshot.get('time') or center_time)
        return snapshot

    @staticmethod
    def _critical_signature(status):
        sources = []
        for name, source in sorted(status.get('active_sources', {}).items()):
            if source.get('level') == 'critical':
                sources.append(name)
        return ','.join(sources)

    def _make_report_id(self, accident_time, signature):
        raw = f'{self.machine_id}-{int(float(accident_time) * 1000)}-{signature or "critical"}'
        safe = ''.join(
            char if char.isalnum() or char in '._-' else '-'
            for char in raw
        )
        return safe[:120]

    def _build_recorded_media(self, center_time, report_id):
        if not self.viewer_base_url:
            return {'recorded': False, 'archive_error': 'viewer disabled'}
        center = float(center_time)
        before = max(0.0, self.video_clip_before)
        after = max(0.0, self.video_clip_after)
        fps = max(1.0, self.video_clip_fps)
        clip_id = report_id
        archive_result = self._archive_video_clip(clip_id, center, before, after, fps)
        media = {
            'clip_id': clip_id,
            'recorded': bool(archive_result.get('ok')),
            'archive_result': archive_result,
            'clip_center_time': center,
            'clip_before_sec': before,
            'clip_after_sec': after,
            'clip_fps': fps,
        }
        if archive_result.get('ok'):
            encoded_id = urllib.parse.quote(clip_id, safe='')
            media['clip_url'] = (
                f'{self.viewer_base_url}/recorded_clip.mjpg?clip_id={encoded_id}'
            )
            media['frame_url_template'] = (
                f'{self.viewer_base_url}/recorded_frame.jpg?'
                f'clip_id={encoded_id}&index={{index}}'
            )
            media['frame_count'] = int(archive_result.get('frame_count') or 0)
        else:
            media['archive_error'] = archive_result.get('error', 'archive failed')
        return media

    def _archive_video_clip(self, clip_id, center, before, after, fps):
        if not self.viewer_base_url:
            return {'ok': False, 'error': 'viewer disabled'}
        params = urllib.parse.urlencode({
            'clip_id': clip_id,
            'center': f'{center:.3f}',
            'before': f'{before:.1f}',
            'after': f'{after:.1f}',
            'fps': f'{fps:.1f}',
        })
        url = f'{self.viewer_base_url}/archive_clip?{params}'
        try:
            with urllib.request.urlopen(url, timeout=8.0) as response:
                data = response.read().decode('utf-8')
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            return {'ok': False, 'error': str(exc)}
        try:
            payload = json.loads(data)
        except json.JSONDecodeError:
            return {'ok': False, 'error': 'invalid archive response'}
        return payload if isinstance(payload, dict) else {'ok': False, 'error': 'invalid archive response'}

    def _build_media(self, center_time):
        if not self.viewer_base_url:
            return {}
        center = float(center_time)
        before = max(0.0, self.video_clip_before)
        after = max(0.0, self.video_clip_after)
        fps = max(1.0, self.video_clip_fps)
        return {
            'viewer_url': f'{self.viewer_base_url}/',
            'snapshot_url': f'{self.viewer_base_url}/snapshot.jpg?t={center:.3f}',
            'stream_url': f'{self.viewer_base_url}/stream.mjpg',
            'clip_url': (
                f'{self.viewer_base_url}/clip.mjpg?'
                f'center={center:.3f}&before={before:.1f}&after={after:.1f}&fps={fps:.1f}'
            ),
            'clip_center_time': center,
            'clip_before_sec': before,
            'clip_after_sec': after,
        }

    def _post_json(self, path, payload):
        if not self.base_station_url:
            return
        url = f'{self.base_station_url}{path}'
        data = json.dumps(payload).encode('utf-8')
        request = urllib.request.Request(
            url,
            data=data,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        try:
            with urllib.request.urlopen(request, timeout=0.6) as response:
                response.read(64)
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            self.get_logger().warning(
                f'Base station post failed: {exc}',
                throttle_duration_sec=5.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = IotSupervisorNode()
    stopping = {'value': False}

    def stop_node(signum, frame):
        del signum, frame
        stopping['value'] = True
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, stop_node)
    signal.signal(signal.SIGINT, stop_node)

    try:
        while rclpy.ok() and not stopping['value']:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.buzzer is not None:
            node.buzzer.write(False)
            if getattr(node.buzzer, 'value_fd', None) is not None:
                os.close(node.buzzer.value_fd)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
