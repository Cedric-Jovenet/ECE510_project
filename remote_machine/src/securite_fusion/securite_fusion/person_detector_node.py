#!/usr/bin/env python3
# Runs YOLO person detection on camera frames and publishes compact JSON boxes
# plus an optional annotated image for calibration.
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class PersonDetectorNode(Node):
    """Detect people in /image_raw with YOLOv8 and publish JSON boxes."""

    def __init__(self):
        super().__init__('person_detector_node')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('detections_topic', '/person_detections')
        self.declare_parameter('annotated_topic', '/person_detections_image')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('max_fps', 2.0)
        self.declare_parameter('publish_annotated', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.annotated_topic = self.get_parameter('annotated_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.image_size = int(self.get_parameter('image_size').value)
        self.device = self.get_parameter('device').value
        self.max_fps = float(self.get_parameter('max_fps').value)
        self.publish_annotated = bool(
            self.get_parameter('publish_annotated').value
        )
        self.min_period = 1.0 / self.max_fps if self.max_fps > 0 else 0.0
        self.last_inference_time = 0.0

        if YOLO is None:
            raise RuntimeError(
                'ultralytics is required for YOLO person detection'
            )

        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        self.model = YOLO(self.model_path)

        self.detections_pub = self.create_publisher(
            String,
            self.detections_topic,
            10,
        )
        self.annotated_pub = self.create_publisher(
            Image,
            self.annotated_topic,
            10,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )
        self.get_logger().info(
            'Listening on '
            f'{self.image_topic}; publishing {self.detections_topic}'
        )

    def image_callback(self, msg):
        now = time.monotonic()
        if now - self.last_inference_time < self.min_period:
            return
        self.last_inference_time = now

        try:
            rgb = self._image_to_rgb(msg)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=2.0)
            return

        results = self.model.predict(
            source=rgb,
            classes=[0],
            conf=self.confidence_threshold,
            imgsz=self.image_size,
            device=self.device,
            verbose=False,
        )
        detections = self._extract_person_boxes(results[0])
        self._publish_detections(msg, detections)

        if self.publish_annotated:
            self._publish_annotated_image(msg, rgb, detections)

    def _image_to_rgb(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8', 'mono8'):
            raise ValueError(f'Unsupported image encoding: {msg.encoding}')

        if msg.encoding == 'mono8':
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height,
                msg.width,
            )
            return cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height,
            msg.width,
            3,
        )
        if msg.encoding == 'bgr8':
            return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image.copy()

    def _extract_person_boxes(self, result):
        detections = []
        if result.boxes is None:
            return detections

        for box in result.boxes:
            xyxy = box.xyxy[0].detach().cpu().numpy().tolist()
            confidence = float(box.conf[0].detach().cpu().item())
            class_id = int(box.cls[0].detach().cpu().item())
            xmin, ymin, xmax, ymax = xyxy
            detections.append({
                'class_id': class_id,
                'class_name': 'person',
                'confidence': confidence,
                'xmin': float(xmin),
                'ymin': float(ymin),
                'xmax': float(xmax),
                'ymax': float(ymax),
                'x_center': float((xmin + xmax) * 0.5),
            })
        return detections

    def _publish_detections(self, image_msg, detections):
        payload = {
            'stamp': {
                'sec': image_msg.header.stamp.sec,
                'nanosec': image_msg.header.stamp.nanosec,
            },
            'image_width': image_msg.width,
            'image_height': image_msg.height,
            'detections': detections,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.detections_pub.publish(msg)

    def _publish_annotated_image(self, image_msg, rgb, detections):
        annotated = rgb.copy()
        for detection in detections:
            pt1 = (int(detection['xmin']), int(detection['ymin']))
            pt2 = (int(detection['xmax']), int(detection['ymax']))
            cv2.rectangle(annotated, pt1, pt2, (0, 255, 0), 2)
            label = f"person {detection['confidence']:.2f}"
            y = max(pt1[1] - 8, 16)
            cv2.putText(
                annotated,
                label,
                (pt1[0], y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        msg = Image()
        msg.header = image_msg.header
        msg.height = image_msg.height
        msg.width = image_msg.width
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = image_msg.width * 3
        msg.data = annotated.tobytes()
        self.annotated_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
