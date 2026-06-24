#!/usr/bin/env python3
import json
import math
import signal
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_ANCHORS = {
    '1': [-0.35, 0.0],
    '2': [0.35, 0.0],
    '3': [0.0, 0.55],
}


class WorkerState:
    def __init__(self):
        self.ranges: Dict[int, dict] = {}
        self.position: Optional[Tuple[float, float]] = None
        self.last_time: Optional[float] = None
        self.velocity = (0.0, 0.0)
        self.display_position: Optional[Tuple[float, float]] = None
        self.display_last_time: Optional[float] = None
        self.display_velocity = (0.0, 0.0)
        self.display_mode = 'none'
        self.display_rejected_count = 0
        self.last_reliable_time: Optional[float] = None
        self.imu_accel = (0.0, 0.0)
        self.imu_bias: Optional[Tuple[float, float]] = None
        self.imu_last_time: Optional[float] = None
        self.imu_prediction_count = 0
        self.reliable_candidates = []
        self.error: Optional[float] = None
        self.position_quality = 'none'
        self.range_count = 0
        self.ambiguous = True
        self.distance_bounds = (None, None)


class UwbPositionNode(Node):
    """Estimate worker 2D positions from UWB ranges to fixed anchors."""

    def __init__(self):
        super().__init__('uwb_position_node')

        self.declare_parameter('ranges_topic', '/uwb/ranges')
        self.declare_parameter('imu_topic', '/uwb/imu')
        self.declare_parameter('workers_topic', '/uwb/workers')
        self.declare_parameter('anchor_positions_json', json.dumps(DEFAULT_ANCHORS))
        self.declare_parameter('range_timeout_sec', 1.5)
        self.declare_parameter('stale_display_sec', 15.0)
        self.declare_parameter('publish_period_sec', 0.1)
        self.declare_parameter('smoothing_alpha', 0.35)
        self.declare_parameter('display_smoothing_alpha', 0.22)
        self.declare_parameter('display_max_speed_mps', 1.4)
        self.declare_parameter('display_jump_tolerance_m', 0.45)
        self.declare_parameter('display_hold_sec', 2.5)
        self.declare_parameter('display_seed_min_reliable_samples', 3)
        self.declare_parameter('display_seed_max_spread_m', 0.8)
        self.declare_parameter('display_seed_window_sec', 2.0)
        self.declare_parameter('imu_prediction_max_sec', 4.0)
        self.declare_parameter('imu_sample_timeout_sec', 1.0)
        self.declare_parameter('imu_accel_deadband_mps2', 0.18)
        self.declare_parameter('imu_max_accel_mps2', 2.5)
        self.declare_parameter('imu_velocity_decay', 0.86)
        self.declare_parameter('imu_x_axis', 'ax')
        self.declare_parameter('imu_y_axis', 'ay')
        self.declare_parameter('imu_x_sign', 1.0)
        self.declare_parameter('imu_y_sign', 1.0)
        self.declare_parameter('max_trilateration_error_m', 1.0)
        self.declare_parameter('critical_radius_m', 2.0)
        self.declare_parameter('warning_radius_m', 2.0)

        self.anchors = self._load_anchors()
        self.range_timeout = float(self.get_parameter('range_timeout_sec').value)
        self.stale_display_sec = float(
            self.get_parameter('stale_display_sec').value
        )
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self.display_smoothing_alpha = float(
            self.get_parameter('display_smoothing_alpha').value
        )
        self.display_max_speed = float(
            self.get_parameter('display_max_speed_mps').value
        )
        self.display_jump_tolerance = float(
            self.get_parameter('display_jump_tolerance_m').value
        )
        self.display_hold_sec = float(
            self.get_parameter('display_hold_sec').value
        )
        self.display_seed_min_reliable_samples = int(
            self.get_parameter('display_seed_min_reliable_samples').value
        )
        self.display_seed_max_spread = float(
            self.get_parameter('display_seed_max_spread_m').value
        )
        self.display_seed_window = float(
            self.get_parameter('display_seed_window_sec').value
        )
        self.imu_prediction_max_sec = float(
            self.get_parameter('imu_prediction_max_sec').value
        )
        self.imu_sample_timeout_sec = float(
            self.get_parameter('imu_sample_timeout_sec').value
        )
        self.imu_accel_deadband = float(
            self.get_parameter('imu_accel_deadband_mps2').value
        )
        self.imu_max_accel = float(
            self.get_parameter('imu_max_accel_mps2').value
        )
        self.imu_velocity_decay = float(
            self.get_parameter('imu_velocity_decay').value
        )
        self.imu_x_axis = str(self.get_parameter('imu_x_axis').value)
        self.imu_y_axis = str(self.get_parameter('imu_y_axis').value)
        self.imu_x_sign = float(self.get_parameter('imu_x_sign').value)
        self.imu_y_sign = float(self.get_parameter('imu_y_sign').value)
        self.max_trilateration_error = float(
            self.get_parameter('max_trilateration_error_m').value
        )
        self.critical_radius = float(
            self.get_parameter('critical_radius_m').value
        )
        self.warning_radius = float(self.get_parameter('warning_radius_m').value)
        self.workers: Dict[str, WorkerState] = {}

        self.publisher = self.create_publisher(
            String,
            self.get_parameter('workers_topic').value,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('ranges_topic').value,
            self.range_callback,
            50,
        )
        self.create_subscription(
            String,
            self.get_parameter('imu_topic').value,
            self.imu_callback,
            50,
        )
        period = float(self.get_parameter('publish_period_sec').value)
        self.timer = self.create_timer(max(0.02, period), self.publish_state)
        self.get_logger().info(f'UWB anchors: {self.anchors}')

    def _load_anchors(self):
        raw = self.get_parameter('anchor_positions_json').value
        data = json.loads(raw)
        anchors = {}
        for key, value in data.items():
            anchor_id = int(key)
            if len(value) != 2:
                raise ValueError(f'Anchor {key} must be [x, y]')
            anchors[anchor_id] = (float(value[0]), float(value[1]))
        if len(anchors) < 3:
            raise ValueError('At least 3 anchor positions are required')
        return anchors

    def range_callback(self, msg):
        try:
            sample = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Invalid UWB JSON: {exc}')
            return

        try:
            worker_id = str(sample.get('worker_id', 'worker1'))
            anchor_id = int(sample['anchor_id'])
            distance_m = float(sample['distance_m'])
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f'Invalid UWB range sample: {exc}')
            return

        if anchor_id not in self.anchors:
            self.get_logger().warning(
                f'Ignoring range for unknown anchor {anchor_id}',
                throttle_duration_sec=2.0,
            )
            return
        if not math.isfinite(distance_m) or distance_m <= 0.0:
            return

        now = float(sample.get('time', time.time()))
        state = self.workers.setdefault(worker_id, WorkerState())
        state.ranges[anchor_id] = {
            'anchor_id': anchor_id,
            'distance_m': distance_m,
            'time': now,
            'port': sample.get('port'),
        }
        self._update_position(worker_id, state, now)

    def imu_callback(self, msg):
        try:
            sample = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Invalid IMU JSON: {exc}')
            return

        worker_id = str(sample.get('worker_id', 'worker4'))
        state = self.workers.setdefault(worker_id, WorkerState())
        now = float(sample.get('time', time.time()))
        self._record_imu(state, sample, now)
        if self._display_needs_prediction(state, now):
            self._predict_display_from_imu(state, now)

    def _update_position(self, worker_id, state, now):
        fresh_ranges = self._fresh_ranges(state, now)
        if not fresh_ranges:
            return

        state.range_count = len(fresh_ranges)
        state.distance_bounds = self._distance_bounds(fresh_ranges)

        if len(fresh_ranges) >= 3:
            result = self._trilaterate(fresh_ranges)
            quality = 'trilaterated'
            ambiguous = False
            if self._ranges_are_inconsistent(result, state.distance_bounds):
                result = self._range_summary_estimate(fresh_ranges)
                quality = 'inconsistent_ranges'
                ambiguous = True
        elif len(fresh_ranges) == 2:
            result = self._bilaterate(fresh_ranges)
            quality = 'partial_2_anchor'
            ambiguous = True
            if self._ranges_are_inconsistent(result, state.distance_bounds):
                result = self._range_summary_estimate(fresh_ranges)
                quality = 'inconsistent_ranges'
        else:
            result = self._single_anchor_estimate(fresh_ranges)
            quality = 'range_only'
            ambiguous = True

        if result is None:
            self.get_logger().warning(
                f'Could not estimate {worker_id}; check anchor geometry',
                throttle_duration_sec=2.0,
            )
            return

        x, y, error = result
        old_position = state.position
        old_time = state.last_time

        should_smooth = (
            old_position is not None
            and quality == state.position_quality
            and quality != 'range_only'
        )
        if not should_smooth:
            smoothed = (x, y)
        else:
            alpha = max(0.0, min(1.0, self.smoothing_alpha))
            smoothed = (
                old_position[0] * (1.0 - alpha) + x * alpha,
                old_position[1] * (1.0 - alpha) + y * alpha,
            )

        if (
            not ambiguous
            and old_position is not None
            and old_time is not None
            and now > old_time
        ):
            dt = max(1e-3, now - old_time)
            state.velocity = (
                (smoothed[0] - old_position[0]) / dt,
                (smoothed[1] - old_position[1]) / dt,
            )
        elif ambiguous:
            state.velocity = (0.0, 0.0)

        state.position = smoothed
        state.last_time = now
        state.error = error
        state.position_quality = quality
        state.ambiguous = ambiguous
        if self._is_reliable_position(quality, error, state.distance_bounds):
            if state.display_position is None:
                self._seed_display_position(state, smoothed, now)
            else:
                state.last_reliable_time = now
                self._update_display_position(state, smoothed, now)
        else:
            state.reliable_candidates = []
            state.display_rejected_count += 1
            self._predict_display_from_imu(state, now)

    def _record_imu(self, state, sample, now):
        try:
            raw_x = float(sample[self.imu_x_axis]) * self.imu_x_sign
            raw_y = float(sample[self.imu_y_axis]) * self.imu_y_sign
        except (KeyError, TypeError, ValueError):
            return
        if not math.isfinite(raw_x) or not math.isfinite(raw_y):
            return

        if state.imu_bias is None:
            state.imu_bias = (raw_x, raw_y)
        else:
            bias_x, bias_y = state.imu_bias
            speed = math.hypot(*state.display_velocity)
            alpha = 0.025 if speed < 0.08 else 0.002
            state.imu_bias = (
                bias_x * (1.0 - alpha) + raw_x * alpha,
                bias_y * (1.0 - alpha) + raw_y * alpha,
            )

        bias_x, bias_y = state.imu_bias
        accel_x = raw_x - bias_x
        accel_y = raw_y - bias_y
        accel_norm = math.hypot(accel_x, accel_y)
        if accel_norm < max(0.0, self.imu_accel_deadband):
            accel_x = 0.0
            accel_y = 0.0
        elif accel_norm > max(0.1, self.imu_max_accel):
            scale = max(0.1, self.imu_max_accel) / accel_norm
            accel_x *= scale
            accel_y *= scale

        state.imu_accel = (accel_x, accel_y)
        state.imu_last_time = now

    def _display_needs_prediction(self, state, now):
        if state.display_position is None or state.display_last_time is None:
            return False
        if state.last_reliable_time is None:
            return False
        if state.ambiguous:
            return True
        if state.last_time is None:
            return False
        return now - state.last_time > self.range_timeout

    def _is_reliable_position(self, quality, error, bounds):
        lower, upper = bounds
        if upper is not None and lower > upper:
            return False
        return (
            quality == 'trilaterated'
            and error is not None
            and error <= self.max_trilateration_error
        )

    def _seed_display_position(self, state, target, now):
        state.reliable_candidates.append((now, target[0], target[1]))
        cutoff = now - max(0.1, self.display_seed_window)
        state.reliable_candidates = [
            item for item in state.reliable_candidates if item[0] >= cutoff
        ]

        min_samples = max(1, self.display_seed_min_reliable_samples)
        if len(state.reliable_candidates) < min_samples:
            state.display_mode = 'waiting_stable_uwb'
            return

        recent = state.reliable_candidates[-min_samples:]
        center_x = sum(item[1] for item in recent) / len(recent)
        center_y = sum(item[2] for item in recent) / len(recent)
        spread = max(
            math.hypot(item[1] - center_x, item[2] - center_y)
            for item in recent
        )
        if spread > max(0.05, self.display_seed_max_spread):
            state.display_mode = 'waiting_stable_uwb'
            return

        state.display_position = (center_x, center_y)
        state.display_last_time = now
        state.display_velocity = (0.0, 0.0)
        state.display_mode = 'uwb_seed'
        state.last_reliable_time = now

    def _update_display_position(self, state, target, now):
        if state.display_position is None or state.display_last_time is None:
            state.display_position = target
            state.display_last_time = now
            state.display_velocity = (0.0, 0.0)
            state.display_mode = 'uwb'
            return

        old_x, old_y = state.display_position
        dx = target[0] - old_x
        dy = target[1] - old_y
        jump = math.hypot(dx, dy)
        dt = max(1e-3, now - state.display_last_time)
        max_speed = max(0.1, self.display_max_speed)
        allowed_jump = max(0.05, self.display_jump_tolerance) + max_speed * dt
        max_step = max(0.03, max_speed * dt)

        if jump > allowed_jump:
            state.display_rejected_count += 1
            ratio = min(1.0, max_step / jump)
            next_x = old_x + dx * ratio
            next_y = old_y + dy * ratio
            state.display_position = (next_x, next_y)
            state.display_velocity = (
                (next_x - old_x) / dt,
                (next_y - old_y) / dt,
            )
            state.display_last_time = now
            state.display_mode = 'uwb_limited'
            return

        alpha = max(0.02, min(1.0, self.display_smoothing_alpha))
        next_x = old_x + dx * alpha
        next_y = old_y + dy * alpha

        step = math.hypot(next_x - old_x, next_y - old_y)
        if step > max_step and step > 0.0:
            ratio = max_step / step
            next_x = old_x + (next_x - old_x) * ratio
            next_y = old_y + (next_y - old_y) * ratio

        state.display_position = (next_x, next_y)
        state.display_velocity = (
            (next_x - old_x) / dt,
            (next_y - old_y) / dt,
        )
        state.display_last_time = now
        state.display_mode = 'uwb'

    def _predict_display_from_imu(self, state, now):
        if state.display_position is None or state.display_last_time is None:
            state.display_mode = 'waiting_reliable_uwb'
            return

        last_reliable = state.last_reliable_time
        if (
            last_reliable is None
            or now - last_reliable > max(0.1, self.imu_prediction_max_sec)
        ):
            state.display_velocity = (0.0, 0.0)
            state.display_last_time = now
            state.display_mode = 'hold'
            return

        dt = now - state.display_last_time
        if dt <= 0.0:
            return
        dt = min(dt, 0.25)

        imu_fresh = (
            state.imu_last_time is not None
            and now - state.imu_last_time <= max(0.1, self.imu_sample_timeout_sec)
        )
        decay = max(0.0, min(1.0, self.imu_velocity_decay))
        vx, vy = state.display_velocity
        if imu_fresh:
            ax, ay = state.imu_accel
            vx = vx * decay + ax * dt
            vy = vy * decay + ay * dt
            speed = math.hypot(vx, vy)
            max_speed = max(0.1, self.display_max_speed)
            if speed > max_speed:
                scale = max_speed / speed
                vx *= scale
                vy *= scale
            old_x, old_y = state.display_position
            state.display_position = (old_x + vx * dt, old_y + vy * dt)
            state.display_velocity = (vx, vy)
            state.display_last_time = now
            state.display_mode = 'imu_prediction'
            state.imu_prediction_count += 1
        else:
            state.display_velocity = (vx * decay, vy * decay)
            state.display_last_time = now
            state.display_mode = 'hold'

    def _fresh_ranges(self, state, now):
        fresh = {}
        for anchor_id, sample in state.ranges.items():
            if now - float(sample['time']) <= self.range_timeout:
                fresh[anchor_id] = sample
        return fresh

    def _distance_bounds(self, ranges):
        lower = 0.0
        upper = math.inf
        for anchor_id, sample in ranges.items():
            ax, ay = self.anchors[anchor_id]
            anchor_radius = math.hypot(ax, ay)
            measured = float(sample['distance_m'])
            lower = max(lower, measured - anchor_radius)
            upper = min(upper, measured + anchor_radius)
        if math.isinf(upper):
            upper = None
        return lower, upper

    def _single_anchor_estimate(self, ranges):
        anchor_id = next(iter(ranges))
        ax, ay = self.anchors[anchor_id]
        distance = float(ranges[anchor_id]['distance_m'])
        # One range defines a circle, not a point. Display it in front of the
        # machine so the worker remains visible while the range circle carries
        # the real measurement.
        return ax, ay + distance, None

    def _range_summary_estimate(self, ranges):
        distances = sorted(float(sample['distance_m']) for sample in ranges.values())
        count = len(distances)
        if count == 0:
            return None
        if count % 2:
            distance = distances[count // 2]
        else:
            distance = 0.5 * (distances[count // 2 - 1] + distances[count // 2])
        spread = distances[-1] - distances[0]
        return 0.0, distance, spread

    def _ranges_are_inconsistent(self, result, bounds):
        if result is None:
            return True
        _, _, error = result
        lower, upper = bounds
        if upper is not None and lower > upper:
            return True
        return error is not None and error > self.max_trilateration_error

    def _bilaterate(self, ranges):
        first_id, second_id = sorted(ranges)[:2]
        x0, y0 = self.anchors[first_id]
        x1, y1 = self.anchors[second_id]
        r0 = float(ranges[first_id]['distance_m'])
        r1 = float(ranges[second_id]['distance_m'])
        dx = x1 - x0
        dy = y1 - y0
        baseline = math.hypot(dx, dy)
        if baseline < 1e-6:
            return self._single_anchor_estimate({first_id: ranges[first_id]})

        ux = dx / baseline
        uy = dy / baseline
        along = (r0 * r0 - r1 * r1 + baseline * baseline) / (2.0 * baseline)
        h2 = r0 * r0 - along * along
        clamped = h2 < 0.0
        height = math.sqrt(max(0.0, h2))

        base_x = x0 + along * ux
        base_y = y0 + along * uy
        px = -uy
        py = ux
        candidates = [
            (base_x + height * px, base_y + height * py),
            (base_x - height * px, base_y - height * py),
        ]
        x, y = max(candidates, key=lambda point: point[1])
        error = math.sqrt(-h2) if clamped else None
        return x, y, error

    def _trilaterate(self, ranges):
        anchor_ids = sorted(ranges)
        ref_id = anchor_ids[0]
        x0, y0 = self.anchors[ref_id]
        d0 = float(ranges[ref_id]['distance_m'])

        rows = []
        values = []
        for anchor_id in anchor_ids[1:]:
            xi, yi = self.anchors[anchor_id]
            di = float(ranges[anchor_id]['distance_m'])
            rows.append([2.0 * (xi - x0), 2.0 * (yi - y0)])
            values.append(
                d0 * d0 - di * di
                - x0 * x0 + xi * xi
                - y0 * y0 + yi * yi
            )

        try:
            solution, _, _, _ = np.linalg.lstsq(
                np.asarray(rows, dtype=np.float64),
                np.asarray(values, dtype=np.float64),
                rcond=None,
            )
        except np.linalg.LinAlgError:
            return None

        x, y = float(solution[0]), float(solution[1])
        residuals = []
        for anchor_id in anchor_ids:
            ax, ay = self.anchors[anchor_id]
            measured = float(ranges[anchor_id]['distance_m'])
            predicted = math.hypot(x - ax, y - ay)
            residuals.append(predicted - measured)
        error = math.sqrt(sum(value * value for value in residuals) / len(residuals))
        return x, y, error

    def publish_state(self):
        now = time.time()
        workers = []
        for worker_id, state in self.workers.items():
            fresh_ranges = self._fresh_ranges(state, now)
            if state.position is None or state.last_time is None:
                continue
            age = now - state.last_time
            if age > self.stale_display_sec:
                continue
            if fresh_ranges:
                displayed_ranges = list(fresh_ranges.values())
                quality = state.position_quality
                stale = False
            else:
                displayed_ranges = list(state.ranges.values())
                quality = f'stale_{state.position_quality}'
                stale = True

            x, y = state.position
            vx, vy = state.velocity
            display_position_valid = state.display_position is not None
            display_x = None
            display_y = None
            if display_position_valid:
                display_x, display_y = state.display_position
            display_vx, display_vy = state.display_velocity
            distance = math.hypot(x, y)
            display_distance = (
                math.hypot(display_x, display_y)
                if display_position_valid
                else None
            )
            imu_age = (
                None
                if state.imu_last_time is None
                else max(0.0, now - state.imu_last_time)
            )
            reliable_age = (
                None
                if state.last_reliable_time is None
                else max(0.0, now - state.last_reliable_time)
            )
            lower_bound, upper_bound = state.distance_bounds
            warning_distance = (
                distance
                if state.position_quality == 'trilaterated'
                else lower_bound
            )
            workers.append({
                'id': worker_id,
                'x': x,
                'y': y,
                'display_x': display_x,
                'display_y': display_y,
                'display_position_valid': display_position_valid,
                'distance_to_machine_m': distance,
                'display_distance_to_machine_m': display_distance,
                'distance_lower_bound_m': lower_bound,
                'distance_upper_bound_m': upper_bound,
                'error_m': getattr(state, 'error', None),
                'age_sec': age,
                'range_count': len(fresh_ranges),
                'last_range_count': len(state.ranges),
                'position_quality': quality,
                'ambiguous': state.ambiguous or stale,
                'stale': stale,
                'display_stabilized': True,
                'display_mode': state.display_mode,
                'display_rejected_count': state.display_rejected_count,
                'last_reliable_age_sec': reliable_age,
                'imu_age_sec': imu_age,
                'imu_prediction_count': state.imu_prediction_count,
                'velocity': {
                    'vx': vx,
                    'vy': vy,
                    'speed_mps': math.hypot(vx, vy),
                },
                'display_velocity': {
                    'vx': display_vx,
                    'vy': display_vy,
                    'speed_mps': math.hypot(display_vx, display_vy),
                },
                'warning_level': self._warning_level(warning_distance),
                'ranges': displayed_ranges,
            })

        payload = {
            'time': now,
            'frame': 'machine',
            'axis': {'x': 'right', 'y': 'front'},
            'anchors': [
                {'id': anchor_id, 'x': pos[0], 'y': pos[1]}
                for anchor_id, pos in sorted(self.anchors.items())
            ],
            'zones': {
                'critical_radius_m': self.critical_radius,
                'warning_radius_m': self.warning_radius,
            },
            'workers': workers,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.publisher.publish(msg)

    def _warning_level(self, distance):
        if distance <= self.critical_radius:
            return 'critical'
        if distance <= self.warning_radius:
            return 'warning'
        return 'clear'


def main(args=None):
    rclpy.init(args=args)
    node = UwbPositionNode()
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
