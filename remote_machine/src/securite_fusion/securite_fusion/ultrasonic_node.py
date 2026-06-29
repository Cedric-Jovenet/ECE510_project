#!/usr/bin/env python3
# Reads GPIO ultrasonic sensors and publishes a single safety level for the
# machine perimeter.
import json
import os
import signal
import statistics
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_SENSORS = [
    {'id': 'hc1', 'trig_gpio': 26, 'echo_gpio': 25, 'enabled': True},
    {'id': 'hc2', 'trig_gpio': 21, 'echo_gpio': 20, 'enabled': True},
]


class SysfsGpio:
    def __init__(self, gpio_base):
        self.gpio_base = int(gpio_base)

    def path_for_bcm(self, bcm_gpio):
        gpio = self.gpio_base + int(bcm_gpio)
        path = f'/sys/class/gpio/gpio{gpio}'
        if not os.path.isdir(path):
            with open('/sys/class/gpio/export', 'w', encoding='utf-8') as handle:
                handle.write(str(gpio))
            time.sleep(0.05)
        return gpio, path

    @staticmethod
    def set_direction(path, direction):
        with open(os.path.join(path, 'direction'), 'w', encoding='utf-8') as handle:
            handle.write(direction)

    @staticmethod
    def write(path, value):
        with open(os.path.join(path, 'value'), 'w', encoding='utf-8') as handle:
            handle.write('1' if value else '0')

    @staticmethod
    def read(path):
        with open(os.path.join(path, 'value'), 'r', encoding='utf-8') as handle:
            return handle.read(1) == '1'


class UltrasonicNode(Node):
    """Poll HC-SR04 sensors and publish filtered distances."""

    def __init__(self):
        super().__init__('ultrasonic_node')

        self.declare_parameter('topic', '/ultrasonic/status')
        self.declare_parameter('gpio_base', 571)
        self.declare_parameter('sensors_json', json.dumps(DEFAULT_SENSORS))
        self.declare_parameter('critical_distance_m', 1.0)
        self.declare_parameter('warning_distance_m', 1.0)
        self.declare_parameter('period_sec', 0.35)
        self.declare_parameter('samples_per_sensor', 5)
        self.declare_parameter('min_valid_samples', 2)
        self.declare_parameter('echo_timeout_sec', 0.07)
        self.declare_parameter('speed_of_sound_mps', 343.0)

        self.publisher = self.create_publisher(
            String,
            self.get_parameter('topic').value,
            10,
        )
        self.gpio = SysfsGpio(self.get_parameter('gpio_base').value)
        self.sensors = self._load_sensors()
        self.critical_distance = float(
            self.get_parameter('critical_distance_m').value
        )
        self.warning_distance = float(
            self.get_parameter('warning_distance_m').value
        )
        self.samples_per_sensor = int(
            self.get_parameter('samples_per_sensor').value
        )
        self.min_valid_samples = int(
            self.get_parameter('min_valid_samples').value
        )
        self.echo_timeout = float(self.get_parameter('echo_timeout_sec').value)
        self.speed_of_sound = float(
            self.get_parameter('speed_of_sound_mps').value
        )

        self._prepare_gpio()
        period = float(self.get_parameter('period_sec').value)
        self.timer = self.create_timer(max(0.1, period), self.tick)
        self.get_logger().info(
            f'Ultrasonic polling active for {[sensor["id"] for sensor in self.sensors]}'
        )

    def _load_sensors(self):
        raw = self.get_parameter('sensors_json').value
        sensors = json.loads(raw)
        return [
            sensor
            for sensor in sensors
            if bool(sensor.get('enabled', True))
        ]

    def _prepare_gpio(self):
        for sensor in self.sensors:
            _, trig_path = self.gpio.path_for_bcm(sensor['trig_gpio'])
            _, echo_path = self.gpio.path_for_bcm(sensor['echo_gpio'])
            self.gpio.set_direction(trig_path, 'out')
            self.gpio.write(trig_path, False)
            self.gpio.set_direction(echo_path, 'in')

    def tick(self):
        now = time.time()
        sensor_status = []
        levels = []

        for sensor in self.sensors:
            status = self._measure_sensor(sensor)
            sensor_status.append(status)
            levels.append(status['level'])

        overall = 'clear'
        if 'critical' in levels:
            overall = 'critical'
        elif 'warning' in levels:
            overall = 'warning'

        payload = {
            'time': now,
            'overall_level': overall,
            'critical_distance_m': self.critical_distance,
            'warning_distance_m': self.warning_distance,
            'sensors': sensor_status,
        }
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'))
        self.publisher.publish(message)

    def _measure_sensor(self, sensor):
        trig_gpio, trig_path = self.gpio.path_for_bcm(sensor['trig_gpio'])
        echo_gpio, echo_path = self.gpio.path_for_bcm(sensor['echo_gpio'])

        distances = []
        errors = []
        idle_high = self.gpio.read(echo_path)
        for _ in range(max(1, self.samples_per_sensor)):
            result = self._single_measurement(trig_path, echo_path)
            if result is None:
                errors.append('timeout')
            else:
                distances.append(result)
            time.sleep(0.02)

        level = 'unknown'
        distance_m = None
        if len(distances) >= max(1, self.min_valid_samples):
            distance_m = float(statistics.median(distances))
            if distance_m <= self.critical_distance:
                level = 'critical'
            elif distance_m <= self.warning_distance:
                level = 'warning'
            else:
                level = 'clear'

        return {
            'id': sensor['id'],
            'trig_gpio': sensor['trig_gpio'],
            'echo_gpio': sensor['echo_gpio'],
            'trig_sysfs': trig_gpio,
            'echo_sysfs': echo_gpio,
            'distance_m': distance_m,
            'level': level,
            'valid_samples': len(distances),
            'sample_count': self.samples_per_sensor,
            'idle_echo_high': idle_high,
            'errors': errors[:3],
        }

    def _single_measurement(self, trig_path, echo_path):
        self.gpio.write(trig_path, False)
        time.sleep(0.002)
        self.gpio.write(trig_path, True)
        time.sleep(0.000012)
        self.gpio.write(trig_path, False)

        deadline = time.monotonic() + self.echo_timeout
        while not self.gpio.read(echo_path) and time.monotonic() < deadline:
            pass
        if time.monotonic() >= deadline:
            return None

        start = time.monotonic()
        deadline = start + self.echo_timeout
        while self.gpio.read(echo_path) and time.monotonic() < deadline:
            pass
        end = time.monotonic()
        if end >= deadline:
            return None

        pulse_sec = end - start
        return pulse_sec * self.speed_of_sound / 2.0


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
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
