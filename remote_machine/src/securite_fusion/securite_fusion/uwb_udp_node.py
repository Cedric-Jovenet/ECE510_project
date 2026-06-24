#!/usr/bin/env python3
"""Receive UWB tag WiFi telemetry and expose it as ROS 2 topics."""

import json
import socket
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, String


class UwbUdpNode(Node):
    """Receive tag telemetry over WiFi UDP and publish ROS messages."""

    def __init__(self):
        """Create publishers and bind the nonblocking UDP socket."""
        super().__init__('uwb_udp_node')
        self.declare_parameter('bind_host', '0.0.0.0')
        self.declare_parameter('port', 8890)
        self.declare_parameter('worker_id', 'worker4')
        self.declare_parameter('max_distance_m', 60.0)
        self.declare_parameter('imu_topic', '/uwb/imu')

        self.worker_id = str(self.get_parameter('worker_id').value)
        self.max_distance_m = float(
            self.get_parameter('max_distance_m').value
        )
        host = str(self.get_parameter('bind_host').value)
        port = int(self.get_parameter('port').value)

        self.ranges_pub = self.create_publisher(String, '/uwb/ranges', 20)
        self.raw_pub = self.create_publisher(
            String, '/uwb/tag_telemetry', 20
        )
        self.node_status_pub = self.create_publisher(
            String, '/uwb/node_status', 20
        )
        self.imu_pub = self.create_publisher(
            Float32MultiArray, '/imu/data', 20
        )
        self.imu_json_pub = self.create_publisher(
            String, self.get_parameter('imu_topic').value, 20
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setblocking(False)
        self.sock.bind((host, port))

        self.last_status_log = {}
        self.timer = self.create_timer(0.01, self.tick)
        self.get_logger().info(
            f'UWB tag UDP receiver listening on {host}:{port}'
        )

    def tick(self):
        """Drain all currently queued UDP datagrams."""
        for _ in range(100):
            try:
                payload, sender = self.sock.recvfrom(4096)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().warning(f'UDP receive failed: {exc}')
                return
            self._handle_packet(payload, sender)

    def _handle_packet(self, payload, sender):
        try:
            data = json.loads(payload.decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(
                f'Ignoring invalid tag UDP packet from {sender[0]}: {exc}'
            )
            return

        if not isinstance(data, dict):
            return

        raw = String()
        raw.data = json.dumps({
            'port': f'udp://{sender[0]}:{sender[1]}',
            'line': json.dumps(data, separators=(',', ':')),
            'time': time.time(),
        }, separators=(',', ':'))
        self.raw_pub.publish(raw)

        packet_type = data.get('type')
        if packet_type == 'uwb_distance':
            self._publish_distance(data, sender)
        elif packet_type == 'imu':
            self._publish_imu(data, sender)
        elif packet_type in ('node_status', 'tag_status'):
            self._publish_node_status(data, sender)
            self._log_status(data, sender)

    def _publish_distance(self, data, sender):
        try:
            anchor_id = int(data['anchor_id'])
            distance_m = float(data['distance_m'])
        except (KeyError, TypeError, ValueError):
            return
        if distance_m <= 0.0 or distance_m > self.max_distance_m:
            return

        sample = {
            'time': time.time(),
            'port': f'udp://{sender[0]}:{sender[1]}',
            'worker_id': str(data.get('worker_id', self.worker_id)),
            'anchor_id': anchor_id,
            'distance_m': distance_m,
            'source_line': json.dumps(data, separators=(',', ':')),
        }
        msg = String()
        msg.data = json.dumps(sample, separators=(',', ':'))
        self.ranges_pub.publish(msg)

    def _publish_imu(self, data, sender):
        try:
            values = [
                float(data[key])
                for key in ('ax', 'ay', 'az', 'gx', 'gy', 'gz')
            ]
        except (KeyError, TypeError, ValueError):
            return
        msg = Float32MultiArray()
        msg.data = values
        self.imu_pub.publish(msg)

        sample = {
            'type': 'imu',
            'time': time.time(),
            'port': f'udp://{sender[0]}:{sender[1]}',
            'worker_id': str(data.get('worker_id', self.worker_id)),
            'ax': values[0],
            'ay': values[1],
            'az': values[2],
            'gx': values[3],
            'gy': values[4],
            'gz': values[5],
            'seq': data.get('seq'),
            'uptime_ms': data.get('uptime_ms'),
        }
        json_msg = String()
        json_msg.data = json.dumps(sample, separators=(',', ':'))
        self.imu_json_pub.publish(json_msg)

    def _log_status(self, data, sender):
        now = time.time()
        node_id = int(data.get('node_id', 4))
        last_log = self.last_status_log.get(node_id, 0.0)
        if now - last_log > 30.0:
            self.last_status_log[node_id] = now
            self.get_logger().info(
                f'UWB node {node_id} telemetry from {sender[0]}: '
                f'wifi={data.get("wifi")} imu={data.get("imu")} '
                f'rssi={data.get("rssi")}'
            )

    def _publish_node_status(self, data, sender):
        status = dict(data)
        status['sender'] = sender[0]
        status['received_at'] = time.time()
        msg = String()
        msg.data = json.dumps(status, separators=(',', ':'))
        self.node_status_pub.publish(msg)

    def destroy_node(self):
        """Close the UDP socket before destroying the ROS node."""
        self.sock.close()
        super().destroy_node()


def main(args=None):
    """Run the UWB UDP receiver until ROS shuts down."""
    rclpy.init(args=args)
    node = UwbUdpNode()
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
