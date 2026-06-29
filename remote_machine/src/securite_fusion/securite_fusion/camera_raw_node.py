#!/usr/bin/env python3
# Publishes raw Raspberry Pi CSI camera frames as ROS Image messages.
# The media-device discovery lives here because Pi 5 `/dev/mediaN` numbering
# can change between boots and camera-stack restarts.
import ctypes
import glob
import re
import signal
import subprocess
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import cv2
except ImportError:  # pragma: no cover - exercised only on stripped systems.
    cv2 = None


class Pi5RawCameraNode(Node):
    """Publish Raspberry Pi 5 CSI IMX219 RAW10 frames as ROS 2 rgb8 images."""

    def __init__(self):
        super().__init__('camera_raw_node')

        self.declare_parameter('device', 'auto')
        self.declare_parameter('media_device', 'auto')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('topic', '/image_raw')
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('stream_buffers', 4)
        self.declare_parameter('bayer_pattern', 'RG')
        self.declare_parameter('auto_enhance', True)
        self.declare_parameter('level_low_percentile', 1.0)
        self.declare_parameter('level_high_percentile', 90.0)
        self.declare_parameter('gamma', 0.55)
        self.declare_parameter('auto_white_balance', True)
        self.declare_parameter('analogue_gain', 120)
        self.declare_parameter('digital_gain', 1024)
        self.declare_parameter('exposure', 1600)
        self.declare_parameter('reconnect_initial_delay_sec', 1.0)
        self.declare_parameter('reconnect_max_delay_sec', 10.0)

        self.device_setting = str(self.get_parameter('device').value)
        self.media_device_setting = str(
            self.get_parameter('media_device').value
        )
        self.device = None
        self.media_device = None
        self.sensor_device = None
        self.csi_device = None
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.stream_buffers = int(self.get_parameter('stream_buffers').value)
        self.bayer_pattern = str(
            self.get_parameter('bayer_pattern').value
        ).upper()
        self.auto_enhance = bool(self.get_parameter('auto_enhance').value)
        self.level_low_percentile = float(
            self.get_parameter('level_low_percentile').value
        )
        self.level_high_percentile = float(
            self.get_parameter('level_high_percentile').value
        )
        self.gamma = float(self.get_parameter('gamma').value)
        self.auto_white_balance = bool(
            self.get_parameter('auto_white_balance').value
        )
        self.analogue_gain = int(self.get_parameter('analogue_gain').value)
        self.digital_gain = int(self.get_parameter('digital_gain').value)
        self.exposure = int(self.get_parameter('exposure').value)
        self.reconnect_initial_delay_sec = max(
            0.1,
            float(self.get_parameter('reconnect_initial_delay_sec').value),
        )
        self.reconnect_max_delay_sec = max(
            self.reconnect_initial_delay_sec,
            float(self.get_parameter('reconnect_max_delay_sec').value),
        )
        self.debayer_code = self._debayer_code()

        self.frame_size = self.width * self.height * 10 // 8
        self.publisher = self.create_publisher(Image, self.topic, 10)
        self._running = threading.Event()
        self._running.set()
        self._process: Optional[subprocess.Popen] = None

        if cv2 is None:
            raise RuntimeError(
                'python3-opencv is required to debayer camera frames'
            )

        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name='pi5-raw-camera-capture',
            daemon=True,
        )
        self._capture_thread.start()
        self.get_logger().info(
            'Camera publisher ready; waiting for IMX219 and publishing '
            f'{self.width}x{self.height} frames on {self.topic}; '
            f'bayer={self.bayer_pattern}, enhance={self.auto_enhance}, '
            'automatic_reconnect=True'
        )

    def _run_command(self, command):
        result = subprocess.run(
            command,
            check=False,
            text=True,
            capture_output=True,
        )
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(
                f'Command failed: {" ".join(command)}\n{detail}'
            )

    def _read_media_topology(self, media_device):
        result = subprocess.run(
            ['media-ctl', '-d', media_device, '-p'],
            check=False,
            text=True,
            capture_output=True,
        )
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(
                f'Command failed: media-ctl -d {media_device} -p\n{detail}'
            )
        return result.stdout

    def _detect_media_device(self):
        # Prefer the rp1-cfe media graph instead of a fixed `/dev/mediaN` path.
        for media_device in sorted(glob.glob('/dev/media*')):
            try:
                topology = self._read_media_topology(media_device)
            except RuntimeError:
                continue
            if re.search(r'^model\s+rp1-cfe$', topology, re.MULTILINE):
                return media_device
        raise RuntimeError('Could not find rp1-cfe media device')

    def _find_entity_id(self, topology, entity_name):
        pattern = re.compile(r'^\s*-\s+entity\s+(\d+):\s+(.+?)\s+\(')
        for line in topology.splitlines():
            match = pattern.match(line)
            if match and match.group(2) == entity_name:
                return match.group(1)
        raise RuntimeError(f'Could not find media entity {entity_name!r}')

    def _find_entity_device(self, topology, entity_name, prefix=False):
        entity_pattern = re.compile(r'^\s*-\s+entity\s+\d+:\s+(.+?)\s+\(')
        device_pattern = re.compile(r'^\s*device node name\s+(\S+)')
        in_entity = False

        for line in topology.splitlines():
            entity_match = entity_pattern.match(line)
            if entity_match:
                name = entity_match.group(1)
                in_entity = (
                    name.startswith(entity_name) if prefix
                    else name == entity_name
                )
                continue
            if in_entity:
                device_match = device_pattern.match(line)
                if device_match:
                    return device_match.group(1)

        raise RuntimeError(
            f'Could not find device node for media entity {entity_name!r}'
        )

    def _configure_camera(self):
        # Configure the sensor, CSI receiver, and video node as one pipeline so
        # v4l2-ctl reads raw frames with matching dimensions and pixel format.
        if self.media_device_setting == 'auto':
            self.media_device = self._detect_media_device()
        else:
            self.media_device = self.media_device_setting
        topology = self._read_media_topology(self.media_device)
        csi_entity = self._find_entity_id(topology, 'csi2')
        ch0_entity = self._find_entity_id(topology, 'rp1-cfe-csi2_ch0')
        self.csi_device = self._find_entity_device(topology, 'csi2')
        self.sensor_device = self._find_entity_device(
            topology,
            'imx219 ',
            prefix=True,
        )
        if self.device_setting == 'auto':
            self.device = self._find_entity_device(
                topology,
                'rp1-cfe-csi2_ch0',
            )
        else:
            self.device = self.device_setting
        self._run_command([
            'media-ctl',
            '-d',
            self.media_device,
            '--links',
            f'{csi_entity}:4->{ch0_entity}:0[1]',
        ])
        for device, pad in (
            (self.sensor_device, 0),
            (self.csi_device, 0),
            (self.csi_device, 4),
        ):
            self._run_command([
                'v4l2-ctl',
                '-d',
                device,
                f'--set-subdev-fmt=pad={pad},width={self.width},'
                f'height={self.height},code=0x300f',
            ])
        self._run_command([
            'v4l2-ctl',
            '-d',
            self.device,
            (
                f'--set-fmt-video=width={self.width},height={self.height},'
                'pixelformat=pRAA'
            ),
        ])
        self._set_sensor_controls()

    def _set_sensor_controls(self):
        controls = []
        if self.exposure >= 0:
            controls.append(f'exposure={self.exposure}')
        if self.analogue_gain >= 0:
            controls.append(f'analogue_gain={self.analogue_gain}')
        if self.digital_gain >= 0:
            controls.append(f'digital_gain={self.digital_gain}')
        if not controls:
            return

        self._run_command([
            'v4l2-ctl',
            '-d',
            self.sensor_device,
            f'--set-ctrl={",".join(controls)}',
        ])

    def _start_capture_process(self):
        command = [
            'v4l2-ctl',
            '-d',
            self.device,
            f'--stream-mmap={self.stream_buffers}',
            '--stream-to=-',
        ]
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
            preexec_fn=_terminate_child_with_parent,
        )
        self._process = process
        threading.Thread(
            target=self._log_capture_stderr,
            args=(process,),
            name='pi5-raw-camera-v4l2-stderr',
            daemon=True,
        ).start()
        return process

    def _log_capture_stderr(self, process):
        if process.stderr is None:
            return
        for line in process.stderr:
            text = line.decode(errors='replace').strip()
            if text:
                self.get_logger().warning(f'v4l2-ctl: {text}')

    def _read_exact_frame(self, process):
        if process.stdout is None:
            return None

        chunks = []
        remaining = self.frame_size
        while self._running.is_set() and remaining > 0:
            chunk = process.stdout.read(remaining)
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return b''.join(chunks)

    def _raw10_to_bayer(self, frame_bytes):
        packed = np.frombuffer(frame_bytes, dtype=np.uint8).reshape((-1, 5))
        pixels = np.empty((packed.shape[0], 4), dtype=np.uint16)
        low_bits = packed[:, 4]
        high_bits = packed[:, :4].astype(np.uint16) << 2
        pixels[:, 0] = high_bits[:, 0] | (low_bits & 0x03)
        pixels[:, 1] = high_bits[:, 1] | ((low_bits >> 2) & 0x03)
        pixels[:, 2] = high_bits[:, 2] | ((low_bits >> 4) & 0x03)
        pixels[:, 3] = high_bits[:, 3] | ((low_bits >> 6) & 0x03)

        return pixels.reshape((self.height, self.width))

    def _raw10_to_rgb(self, frame_bytes):
        bayer10 = self._raw10_to_bayer(frame_bytes)
        rgb16 = cv2.cvtColor(bayer10, self.debayer_code)
        if self.auto_enhance:
            return self._enhance_rgb(rgb16)
        return np.clip(rgb16 >> 2, 0, 255).astype(np.uint8)

    def _enhance_rgb(self, rgb16):
        rgb = rgb16.astype(np.float32)
        low = np.percentile(rgb, self.level_low_percentile)
        high = np.percentile(rgb, self.level_high_percentile)
        if high <= low + 1.0:
            low = float(rgb.min())
            high = float(rgb.max() + 1.0)

        rgb = np.clip((rgb - low) * (255.0 / (high - low)), 0.0, 255.0)
        if self.auto_white_balance:
            means = np.maximum(rgb.reshape(-1, 3).mean(axis=0), 1.0)
            gray = float(np.mean(means))
            gains = np.clip(gray / means, 0.45, 2.8)
            rgb *= gains.reshape((1, 1, 3))

        gamma = max(0.1, self.gamma)
        rgb = np.clip(rgb, 0.0, 255.0) / 255.0
        rgb = np.power(rgb, gamma) * 255.0
        return np.clip(rgb, 0.0, 255.0).astype(np.uint8)

    def _debayer_code(self):
        codes = {
            'RG': cv2.COLOR_BayerRG2RGB,
            'RGGB': cv2.COLOR_BayerRG2RGB,
            'BG': cv2.COLOR_BayerBG2RGB,
            'BGGR': cv2.COLOR_BayerBG2RGB,
            'GR': cv2.COLOR_BayerGR2RGB,
            'GRBG': cv2.COLOR_BayerGR2RGB,
            'GB': cv2.COLOR_BayerGB2RGB,
            'GBRG': cv2.COLOR_BayerGB2RGB,
        }
        if self.bayer_pattern not in codes:
            valid = ', '.join(sorted(codes))
            raise RuntimeError(
                f'Invalid bayer_pattern {self.bayer_pattern}; use {valid}'
            )
        return codes[self.bayer_pattern]

    def _stop_capture_process(self, process=None):
        process = process or self._process
        if process is None:
            return
        if self._process is process:
            self._process = None
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        if process.stdout is not None:
            process.stdout.close()
        if process.stderr is not None:
            process.stderr.close()

    def _wait_for_retry(self, delay_sec):
        deadline = time.monotonic() + delay_sec
        while self._running.is_set() and time.monotonic() < deadline:
            time.sleep(max(0.0, min(0.2, deadline - time.monotonic())))

    def _capture_loop(self):
        # Restart v4l2-ctl on failure instead of exiting the ROS node. The rest
        # of the safety stack can keep running while the camera recovers.
        ever_published = False
        retry_delay = self.reconnect_initial_delay_sec
        while self._running.is_set():
            process = None
            session_frames = 0
            try:
                self._configure_camera()
                process = self._start_capture_process()
                self.get_logger().info(
                    f'Camera capture started on {self.device} via '
                    f'{self.media_device}'
                )

                while self._running.is_set():
                    frame = self._read_exact_frame(process)
                    if frame is None:
                        raise RuntimeError('Camera stream ended unexpectedly')

                    rgb = self._raw10_to_rgb(frame)
                    msg = Image()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.frame_id
                    msg.height = self.height
                    msg.width = self.width
                    msg.encoding = 'rgb8'
                    msg.is_bigendian = False
                    msg.step = self.width * 3
                    msg.data = rgb.tobytes()
                    self.publisher.publish(msg)

                    session_frames += 1
                    retry_delay = self.reconnect_initial_delay_sec
                    if session_frames == 1:
                        if ever_published:
                            self.get_logger().info(
                                'Camera stream recovered; publishing frames again'
                            )
                        else:
                            self.get_logger().info(
                                'First camera frame published'
                            )
                            ever_published = True
            except Exception as exc:
                if self._running.is_set():
                    self.get_logger().warning(
                        f'Camera unavailable: {exc}; retrying in '
                        f'{retry_delay:.1f}s'
                    )
            finally:
                self._stop_capture_process(process)

            if self._running.is_set():
                self._wait_for_retry(retry_delay)
                retry_delay = min(
                    self.reconnect_max_delay_sec,
                    retry_delay * 2.0,
                )

    def destroy_node(self):
        self._running.clear()
        self._stop_capture_process()
        if self._capture_thread.is_alive():
            self._capture_thread.join(timeout=3)
        super().destroy_node()


def _terminate_child_with_parent():
    libc = ctypes.CDLL('libc.so.6')
    pr_set_pdeathsig = 1
    libc.prctl(pr_set_pdeathsig, signal.SIGKILL)


def main(args=None):
    rclpy.init(args=args)
    node = Pi5RawCameraNode()
    stopping = {'value': False}
    destroyed = {'value': False}

    def stop_node(signum, frame):
        del signum, frame
        if stopping['value']:
            return
        stopping['value'] = True
        node.get_logger().info('Shutdown requested; stopping camera stream')
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
        if not destroyed['value']:
            destroyed['value'] = True
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
