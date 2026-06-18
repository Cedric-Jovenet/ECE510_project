#!/usr/bin/env python3
import signal
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class UsbCameraNode(Node):
    """Publish a standard USB/UVC webcam as ROS 2 rgb8 images."""

    def __init__(self):
        super().__init__('usb_camera_node')

        self.declare_parameter('device', '/dev/video8')
        self.declare_parameter('topic', '/image_raw')
        self.declare_parameter('frame_id', 'usb_camera')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('fourcc', 'MJPG')

        self.device = self.get_parameter('device').value
        self.topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.fourcc = self.get_parameter('fourcc').value

        self.publisher = self.create_publisher(Image, self.topic, 10)
        self.capture = self._open_capture()
        self.stopping = False
        self.timer = self.create_timer(1.0 / max(self.fps, 1.0), self.tick)
        self.frame_count = 0
        self.last_warning_time = 0.0

        self.get_logger().info(
            f'Publishing USB camera {self.device} on {self.topic} '
            f'at {self.width}x{self.height}@{self.fps:.1f}'
        )

    def _open_capture(self):
        capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise RuntimeError(f'Unable to open USB camera {self.device}')

        if self.fourcc:
            capture.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*self.fourcc[:4]),
            )
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)

        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if actual_width != self.width or actual_height != self.height:
            self.get_logger().warning(
                f'Requested {self.width}x{self.height}, got '
                f'{actual_width}x{actual_height}'
            )
            self.width = actual_width
            self.height = actual_height

        return capture

    def tick(self):
        if self.stopping:
            return

        ok, frame_bgr = self.capture.read()
        if not ok or frame_bgr is None:
            now = time.monotonic()
            if now - self.last_warning_time > 2.0:
                self.get_logger().warning('USB camera frame read failed')
                self.last_warning_time = now
            return

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = frame_rgb.shape[0]
        msg.width = frame_rgb.shape[1]
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = msg.width * 3
        msg.data = frame_rgb.tobytes()
        self.publisher.publish(msg)

        self.frame_count += 1
        if self.frame_count == 1:
            self.get_logger().info('First USB camera frame published')

    def destroy_node(self):
        self.stopping = True
        if getattr(self, 'capture', None) is not None:
            self.capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    stopping = {'value': False}

    def stop_node(signum, frame):
        del signum, frame
        stopping['value'] = True
        node.stopping = True
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
