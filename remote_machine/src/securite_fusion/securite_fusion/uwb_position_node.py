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
        self.declare_parameter('workers_topic', '/uwb/workers')
        self.declare_parameter('anchor_positions_json', json.dumps(DEFAULT_ANCHORS))
        self.declare_parameter('range_timeout_sec', 1.5)
        self.declare_parameter('stale_display_sec', 15.0)
        self.declare_parameter('publish_period_sec', 0.1)
        self.declare_parameter('smoothing_alpha', 0.35)
        self.declare_parameter('max_trilateration_error_m', 1.0)
        self.declare_parameter('critical_radius_m', 1.5)
        self.declare_parameter('warning_radius_m', 3.0)

        self.anchors = self._load_anchors()
        self.range_timeout = float(self.get_parameter('range_timeout_sec').value)
        self.stale_display_sec = float(
            self.get_parameter('stale_display_sec').value
        )
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
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
            distance = math.hypot(x, y)
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
                'distance_to_machine_m': distance,
                'distance_lower_bound_m': lower_bound,
                'distance_upper_bound_m': upper_bound,
                'error_m': getattr(state, 'error', None),
                'age_sec': age,
                'range_count': len(fresh_ranges),
                'last_range_count': len(state.ranges),
                'position_quality': quality,
                'ambiguous': state.ambiguous or stale,
                'stale': stale,
                'velocity': {
                    'vx': vx,
                    'vy': vy,
                    'speed_mps': math.hypot(vx, vy),
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
