#!/usr/bin/env python3
import ctypes
import signal
import subprocess
import threading
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

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('media_device', '/dev/media3')
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

        self.device = self.get_parameter('device').value
        self.media_device = self.get_parameter('media_device').value
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

        self._configure_camera()
        self._start_capture_process()

        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name='pi5-raw-camera-capture',
            daemon=True,
        )
        self._capture_thread.start()
        self.get_logger().info(
            'Publishing '
            f'{self.width}x{self.height} IMX219 frames on {self.topic}; '
            f'bayer={self.bayer_pattern}, enhance={self.auto_enhance}'
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

    def _configure_camera(self):
        self._run_command([
            'media-ctl',
            '-d',
            self.media_device,
            '--links',
            '"csi2":4->"rp1-cfe-csi2_ch0":0[1]',
        ])
        for device, pad in (
            ('/dev/v4l-subdev2', 0),
            ('/dev/v4l-subdev0', 0),
            ('/dev/v4l-subdev0', 4),
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
            '/dev/v4l-subdev2',
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
        self._process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
            preexec_fn=_terminate_child_with_parent,
        )
        threading.Thread(
            target=self._log_capture_stderr,
            name='pi5-raw-camera-v4l2-stderr',
            daemon=True,
        ).start()

    def _log_capture_stderr(self):
        if self._process is None or self._process.stderr is None:
            return
        for line in self._process.stderr:
            text = line.decode(errors='replace').strip()
            if text:
                self.get_logger().warning(f'v4l2-ctl: {text}')

    def _read_exact_frame(self):
        if self._process is None or self._process.stdout is None:
            return None

        chunks = []
        remaining = self.frame_size
        while self._running.is_set() and remaining > 0:
            chunk = self._process.stdout.read(remaining)
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

    def _capture_loop(self):
        frame_count = 0
        while self._running.is_set():
            frame = self._read_exact_frame()
            if frame is None:
                if self._running.is_set():
                    self.get_logger().error('Camera stream ended unexpectedly')
                    rclpy.shutdown()
                return

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

            frame_count += 1
            if frame_count == 1:
                self.get_logger().info('First camera frame published')

    def destroy_node(self):
        self._running.clear()
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._process.kill()
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
