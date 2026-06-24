#!/usr/bin/env python3
import json
import math
import signal
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String


class FusionNode(Node):
    """Fuse frontal LiDAR obstacles with YOLO person detections."""

    def __init__(self):
        super().__init__('fusion_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('detections_topic', '/person_detections')
        self.declare_parameter('overlay_topic', '/fusion_overlay')
        self.declare_parameter('safety_threshold_m', 2.0)
        self.declare_parameter('lidar_front_angle_deg', 0.0)
        self.declare_parameter('lidar_cone_half_angle_deg', 30.0)
        self.declare_parameter('camera_horizontal_fov_deg', 78.0)
        self.declare_parameter('camera_yaw_offset_deg', 0.0)
        self.declare_parameter('bbox_x_margin_px', 6.0)
        self.declare_parameter('person_distance_percentile', 20.0)
        self.declare_parameter('invert_lidar_x_axis', True)
        self.declare_parameter('status_period_sec', 1.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.overlay_topic = self.get_parameter('overlay_topic').value

        self.safety_threshold = float(
            self.get_parameter('safety_threshold_m').value
        )
        self.lidar_front_angle = math.radians(
            float(self.get_parameter('lidar_front_angle_deg').value)
        )
        self.lidar_cone_half_angle = math.radians(
            float(self.get_parameter('lidar_cone_half_angle_deg').value)
        )
        self.camera_horizontal_fov = math.radians(
            float(self.get_parameter('camera_horizontal_fov_deg').value)
        )
        self.camera_yaw_offset = math.radians(
            float(self.get_parameter('camera_yaw_offset_deg').value)
        )
        self.bbox_x_margin_px = float(
            self.get_parameter('bbox_x_margin_px').value
        )
        self.person_distance_percentile = float(
            self.get_parameter('person_distance_percentile').value
        )
        self.invert_lidar_x_axis = bool(
            self.get_parameter('invert_lidar_x_axis').value
        )
        self.status_period = float(
            self.get_parameter('status_period_sec').value
        )

        self.latest_detections = []
        self.latest_image_size = None
        self.latest_obstacle = None
        self.latest_person_match = None
        self.scan_count = 0
        self.detection_count = 0
        self.last_scan_time = None
        self.last_detection_time = None
        self.last_image_time = None

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )
        self.create_subscription(
            String,
            self.detections_topic,
            self.detections_callback,
            10,
        )
        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )
        self.overlay_pub = self.create_publisher(Image, self.overlay_topic, 10)
        self.status_pub = self.create_publisher(String, '/fusion_status', 10)
        self.status_timer = self.create_timer(
            max(0.2, self.status_period),
            self.status_callback,
        )

        self.get_logger().info(
            'Fusion active: '
            'LiDAR cone '
            f'+/-{math.degrees(self.lidar_cone_half_angle):.1f} deg, '
            f'camera FOV {math.degrees(self.camera_horizontal_fov):.1f} deg'
        )

    def detections_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Invalid detection JSON: {exc}')
            return

        self.latest_detections = payload.get('detections', [])
        self.detection_count = len(self.latest_detections)
        self.last_detection_time = self.get_clock().now()
        width = payload.get('image_width')
        height = payload.get('image_height')
        if width is not None and height is not None:
            self.latest_image_size = (int(width), int(height))

    def scan_callback(self, msg):
        self.scan_count += 1
        self.last_scan_time = self.get_clock().now()
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            return

        indices = np.arange(ranges.size, dtype=np.float32)
        angles = msg.angle_min + indices * msg.angle_increment
        relative_angles = self._normalize_angle(
            angles - self.lidar_front_angle
        )

        valid_mask = (
            np.isfinite(ranges)
            & (ranges > msg.range_min)
            & (ranges < msg.range_max)
            & (np.abs(relative_angles) <= self.lidar_cone_half_angle)
        )
        if not np.any(valid_mask):
            self.latest_obstacle = None
            self.latest_person_match = None
            return

        image_width = self._current_image_width()
        pixels = self.angles_to_pixels(relative_angles, image_width)
        self.latest_obstacle = self._nearest_projection(
            ranges,
            relative_angles,
            pixels,
            valid_mask,
        )
        self.latest_person_match = self._nearest_person_projection(
            ranges,
            relative_angles,
            pixels,
            valid_mask,
            image_width,
        )

        person = self.latest_person_match
        obstacle = self.latest_obstacle
        if person is not None and person['critical']:
            self.get_logger().warn(
                'ALERTE CRITIQUE : humain detecte a '
                f"{person['distance']:.2f} m, angle "
                f"{math.degrees(person['angle']):.1f} deg, "
                f"x={person['pixel_x']:.0f}",
                throttle_duration_sec=0.5,
            )
        elif obstacle is not None and obstacle['critical']:
            self.get_logger().warn(
                'Obstacle critique sans personne YOLO associee : '
                f"{obstacle['distance']:.2f} m, angle "
                f"{math.degrees(obstacle['angle']):.1f} deg, "
                f"x={obstacle['pixel_x']:.0f}"
                if obstacle['pixel_x'] is not None
                else (
                    'Obstacle critique hors FOV camera : '
                    f"{obstacle['distance']:.2f} m, angle "
                    f"{math.degrees(obstacle['angle']):.1f} deg"
                ),
                throttle_duration_sec=0.5,
            )
        elif obstacle is not None:
            self.get_logger().info(
                f"Distance minimale dans le cone : "
                f"{obstacle['distance']:.2f} m",
                throttle_duration_sec=1.0,
            )

    def image_callback(self, msg):
        self.last_image_time = self.get_clock().now()
        self.latest_image_size = (msg.width, msg.height)
        if self.overlay_pub.get_subscription_count() == 0:
            return

        try:
            image = self._image_to_rgb(msg)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=2.0)
            return

        overlay = self._draw_overlay(image)
        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = 'rgb8'
        out.is_bigendian = False
        out.step = msg.width * 3
        out.data = overlay.tobytes()
        self.overlay_pub.publish(out)

    def status_callback(self):
        status = self._build_status()
        message = String()
        message.data = json.dumps(status, separators=(',', ':'))
        self.status_pub.publish(message)

        if status['scan_count'] == 0:
            self.get_logger().warning(
                'En attente de /scan : aucun LaserScan recu. '
                'Verifie que le RPLidar est lance.',
                throttle_duration_sec=max(1.0, self.status_period),
            )
            return

        if self.latest_obstacle is None:
            self.get_logger().info(
                'LiDAR actif, mais aucune mesure valide dans le cone frontal '
                f'+/-{math.degrees(self.lidar_cone_half_angle):.1f} deg',
                throttle_duration_sec=max(1.0, self.status_period),
            )
            return

        person = self.latest_person_match
        if person is not None:
            raw_min = person.get('min_distance')
            raw_text = (
                ''
                if raw_min is None
                else f', min brut={raw_min:.2f} m'
            )
            self.get_logger().info(
                'Distance personne LiDAR-YOLO: '
                f"{person['distance']:.2f} m a "
                f"{math.degrees(person['angle']):+.1f} deg, "
                f"x={person['pixel_x']:.0f}px{raw_text}",
                throttle_duration_sec=max(1.0, self.status_period),
            )
            return

        obstacle = self.latest_obstacle
        pixel = obstacle.get('pixel_x')
        pixel_text = 'hors FOV camera' if pixel is None else f'x={pixel:.0f}px'
        self.get_logger().info(
            'Distance obstacle LiDAR: '
            f"{obstacle['distance']:.2f} m a "
            f"{math.degrees(obstacle['angle']):+.1f} deg, "
            f'{pixel_text}, detections={self.detection_count}, humain=non',
            throttle_duration_sec=max(1.0, self.status_period),
        )

    def _build_status(self):
        obstacle_status = self._projection_status(self.latest_obstacle)
        person_status = self._projection_status(self.latest_person_match)

        return {
            'scan_count': self.scan_count,
            'detections': self.detection_count,
            'image_size': self.latest_image_size,
            'safety_threshold_m': self.safety_threshold,
            'lidar_cone_half_angle_deg': math.degrees(
                self.lidar_cone_half_angle
            ),
            'camera_horizontal_fov_deg': math.degrees(
                self.camera_horizontal_fov
            ),
            'camera_yaw_offset_deg': math.degrees(self.camera_yaw_offset),
            'lidar_front_angle_deg': math.degrees(self.lidar_front_angle),
            'person_distance_percentile': self.person_distance_percentile,
            'nearest_obstacle': obstacle_status,
            'person_match': person_status,
            'obstacle': person_status if person_status else obstacle_status,
        }

    def _projection_status(self, projection):
        if projection is None:
            return None

        detection = projection.get('detection')
        detection_status = None
        if detection is not None:
            detection_status = {
                'xmin': detection.get('xmin'),
                'xmax': detection.get('xmax'),
                'confidence': detection.get('confidence'),
            }

        status = {
            'distance_m': projection['distance'],
            'angle_deg': math.degrees(projection['angle']),
            'pixel_x': projection['pixel_x'],
            'critical': projection['critical'],
            'matched_person': detection is not None,
        }
        if projection.get('min_distance') is not None:
            status['raw_min_distance_m'] = projection['min_distance']
        if detection_status is not None:
            status['detection'] = detection_status
        return status

    def _nearest_projection(self, ranges, relative_angles, pixels, valid_mask):
        valid_indices = np.flatnonzero(valid_mask)
        nearest_valid_offset = int(np.argmin(ranges[valid_mask]))
        nearest_index = int(valid_indices[nearest_valid_offset])
        return self._projection_from_index(
            ranges,
            relative_angles,
            pixels,
            nearest_index,
            detection=None,
        )

    def _nearest_person_projection(
        self,
        ranges,
        relative_angles,
        pixels,
        valid_mask,
        image_width,
    ):
        visible_mask = valid_mask & np.isfinite(pixels)
        if not np.any(visible_mask):
            return None

        best_projection = None
        for detection in self.latest_detections:
            xmin, xmax = self._detection_x_bounds(detection, image_width)
            person_mask = visible_mask & (pixels >= xmin) & (pixels <= xmax)
            if not np.any(person_mask):
                continue

            person_ranges = ranges[person_mask]
            percentile = float(
                np.clip(self.person_distance_percentile, 0.0, 100.0)
            )
            target_distance = float(np.percentile(person_ranges, percentile))
            person_indices = np.flatnonzero(person_mask)
            selected_offset = int(
                np.argmin(np.abs(person_ranges - target_distance))
            )
            selected_index = int(person_indices[selected_offset])
            projection = self._projection_from_index(
                ranges,
                relative_angles,
                pixels,
                selected_index,
                detection=detection,
                min_distance=float(np.min(person_ranges)),
            )
            if (
                best_projection is None
                or projection['distance'] < best_projection['distance']
            ):
                best_projection = projection

        return best_projection

    def _projection_from_index(
        self,
        ranges,
        relative_angles,
        pixels,
        index,
        detection=None,
        min_distance=None,
    ):
        distance = float(ranges[index])
        return {
            'distance': distance,
            'angle': float(relative_angles[index]),
            'pixel_x': (
                None
                if not np.isfinite(pixels[index])
                else float(pixels[index])
            ),
            'critical': distance < self.safety_threshold,
            'detection': detection,
            'min_distance': min_distance,
        }

    def _detection_x_bounds(self, detection, image_width):
        xmin = float(detection.get('xmin', 0.0)) - self.bbox_x_margin_px
        xmax = float(detection.get('xmax', 0.0)) + self.bbox_x_margin_px
        xmin = max(0.0, min(float(image_width - 1), xmin))
        xmax = max(0.0, min(float(image_width - 1), xmax))
        if xmin > xmax:
            xmin, xmax = xmax, xmin
        return xmin, xmax

    def angle_to_pixel(self, lidar_relative_angle, image_width):
        camera_relative = self._normalize_angle(
            lidar_relative_angle - self.camera_yaw_offset
        )
        half_fov = self.camera_horizontal_fov * 0.5
        if abs(camera_relative) > half_fov:
            return None

        if self.invert_lidar_x_axis:
            normalized_x = 0.5 - camera_relative / self.camera_horizontal_fov
        else:
            normalized_x = 0.5 + camera_relative / self.camera_horizontal_fov
        return float(normalized_x * image_width)

    def angles_to_pixels(self, lidar_relative_angles, image_width):
        camera_relative = self._normalize_angle(
            lidar_relative_angles - self.camera_yaw_offset
        )
        half_fov = self.camera_horizontal_fov * 0.5
        pixels = np.full(camera_relative.shape, np.nan, dtype=np.float32)
        visible = np.abs(camera_relative) <= half_fov
        if self.invert_lidar_x_axis:
            normalized_x = 0.5 - camera_relative / self.camera_horizontal_fov
        else:
            normalized_x = 0.5 + camera_relative / self.camera_horizontal_fov
        pixels[visible] = normalized_x[visible] * image_width
        return pixels

    def _find_matching_person(self, pixel_x) -> Optional[dict]:
        if pixel_x is None:
            return None
        for detection in self.latest_detections:
            xmin = float(detection.get('xmin', 0.0)) - self.bbox_x_margin_px
            xmax = float(detection.get('xmax', 0.0)) + self.bbox_x_margin_px
            if xmin <= pixel_x <= xmax:
                return detection
        return None

    def _current_image_width(self):
        if self.latest_image_size is not None:
            return self.latest_image_size[0]
        return 640

    def _draw_overlay(self, image):
        annotated = image.copy()
        height, width = annotated.shape[:2]

        self._draw_visible_lidar_cone(annotated, width, height)
        self._draw_camera_center(annotated, width, height)
        self._draw_person_boxes(annotated)
        self._draw_obstacle_projection(annotated, width, height)

        cv2.putText(
            annotated,
            self._status_text(),
            (12, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return annotated

    def _draw_visible_lidar_cone(self, image, width, height):
        half_fov = self.camera_horizontal_fov * 0.5
        camera_left = self.camera_yaw_offset - half_fov
        camera_right = self.camera_yaw_offset + half_fov
        cone_left = -self.lidar_cone_half_angle
        cone_right = self.lidar_cone_half_angle

        overlap_left = max(camera_left, cone_left)
        overlap_right = min(camera_right, cone_right)
        if overlap_left >= overlap_right:
            return

        x1 = self.angle_to_pixel(overlap_left, width)
        x2 = self.angle_to_pixel(overlap_right, width)
        if x1 is None or x2 is None:
            return

        left_px = int(max(0, min(width - 1, min(x1, x2))))
        right_px = int(max(0, min(width - 1, max(x1, x2))))
        tint = image.copy()
        cv2.rectangle(
            tint,
            (left_px, 0),
            (right_px, height),
            (255, 220, 0),
            -1,
        )
        cv2.addWeighted(tint, 0.18, image, 0.82, 0, dst=image)
        cv2.line(image, (left_px, 0), (left_px, height), (255, 220, 0), 2)
        cv2.line(image, (right_px, 0), (right_px, height), (255, 220, 0), 2)

    def _draw_camera_center(self, image, width, height):
        center_x = int(width * 0.5)
        cv2.line(image, (center_x, 0), (center_x, height), (0, 180, 255), 1)

    def _draw_person_boxes(self, image):
        for detection in self.latest_detections:
            xmin = int(detection.get('xmin', 0))
            ymin = int(detection.get('ymin', 0))
            xmax = int(detection.get('xmax', 0))
            ymax = int(detection.get('ymax', 0))
            conf = float(detection.get('confidence', 0.0))
            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            label = f'person {conf:.2f}'
            if (
                self.latest_person_match is not None
                and self.latest_person_match.get('detection') is detection
            ):
                label += f" {self.latest_person_match['distance']:.2f}m"
            cv2.putText(
                image,
                label,
                (xmin, max(16, ymin - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

    def _draw_obstacle_projection(self, image, width, height):
        if self.latest_obstacle is not None:
            color = (255, 0, 0) if self.latest_obstacle['critical'] else (
                255,
                255,
                255,
            )
            label_prefix = 'raw' if self.latest_person_match is not None else 'obs'
            self._draw_projection(
                image,
                width,
                height,
                self.latest_obstacle,
                color,
                f"{label_prefix} {self.latest_obstacle['distance']:.2f} m",
            )

        if self.latest_person_match is not None:
            self._draw_projection(
                image,
                width,
                height,
                self.latest_person_match,
                (255, 0, 255),
                f"person {self.latest_person_match['distance']:.2f} m",
            )

    def _draw_projection(self, image, width, height, projection, color, label):
        pixel_x = projection.get('pixel_x')
        if pixel_x is None:
            return

        x = int(max(0, min(width - 1, pixel_x)))
        cv2.line(image, (x, 0), (x, height), color, 2)
        cv2.circle(image, (x, height // 2), 8, color, -1)
        cv2.putText(
            image,
            label,
            (min(width - 120, x + 8), height // 2 - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    def _status_text(self):
        return (
            f'cone +/-{math.degrees(self.lidar_cone_half_angle):.0f} deg | '
            f'cam FOV {math.degrees(self.camera_horizontal_fov):.1f} deg | '
            f'yaw {math.degrees(self.camera_yaw_offset):+.1f} deg'
        )

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

    @staticmethod
    def _normalize_angle(angle):
        return np.arctan2(np.sin(angle), np.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
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
