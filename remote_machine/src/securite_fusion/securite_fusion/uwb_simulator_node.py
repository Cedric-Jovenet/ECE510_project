#!/usr/bin/env python3
import json
import math
import signal
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from securite_fusion.uwb_position_node import DEFAULT_ANCHORS


class UwbSimulatorNode(Node):
    """Publish synthetic UWB ranges for testing the 2D map without hardware."""

    def __init__(self):
        super().__init__('uwb_simulator_node')

        self.declare_parameter('ranges_topic', '/uwb/ranges')
        self.declare_parameter('anchor_positions_json', json.dumps(DEFAULT_ANCHORS))
        self.declare_parameter('worker_id', 'worker1')
        self.declare_parameter('publish_period_sec', 0.1)
        self.declare_parameter('noise_m', 0.03)

        self.anchors = {
            int(key): (float(value[0]), float(value[1]))
            for key, value in json.loads(
                self.get_parameter('anchor_positions_json').value
            ).items()
        }
        self.worker_id = self.get_parameter('worker_id').value
        self.noise = float(self.get_parameter('noise_m').value)
        self.start_time = time.time()
        self.publisher = self.create_publisher(
            String,
            self.get_parameter('ranges_topic').value,
            10,
        )
        period = float(self.get_parameter('publish_period_sec').value)
        self.timer = self.create_timer(max(0.02, period), self.tick)
        self.get_logger().info('UWB simulator active')

    def tick(self):
        now = time.time()
        t = now - self.start_time
        x = 0.85 * math.sin(t * 0.45)
        y = 1.15 + 0.65 * math.cos(t * 0.33)

        for anchor_id, (ax, ay) in sorted(self.anchors.items()):
            base = math.hypot(x - ax, y - ay)
            noise = self.noise * math.sin(t * 1.7 + anchor_id)
            msg = String()
            msg.data = json.dumps({
                'time': now,
                'worker_id': self.worker_id,
                'anchor_id': anchor_id,
                'distance_m': max(0.05, base + noise),
                'simulated': True,
            }, separators=(',', ':'))
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UwbSimulatorNode()
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
