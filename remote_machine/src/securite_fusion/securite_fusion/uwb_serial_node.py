#!/usr/bin/env python3
import glob
import array
import fcntl
import json
import os
import re
import select
import signal
import termios
import time
from typing import Dict, Iterable, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DISTANCES_RE = re.compile(
    r'A(?P<anchor>\d+)\s*:\s*(?P<value>[0-9]+(?:\.[0-9]+)?)\s*cm',
    re.IGNORECASE,
)
KEY_VALUE_RE = re.compile(
    r'(?:anchor|a)(?P<anchor>\d+)\s*[=:]\s*'
    r'(?P<value>[0-9]+(?:\.[0-9]+)?)\s*(?P<unit>cm|m)?',
    re.IGNORECASE,
)


class SerialLineReader:
    """Small non-pyserial line reader for USB CDC/UART devices."""

    def __init__(self, path, baudrate):
        self.path = path
        self.baudrate = baudrate
        self.fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        self._configure()
        self._release_modem_control_lines()
        self.buffer = bytearray()

    def _configure(self):
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        speed = self._baud_constant(self.baudrate)
        attrs[4] = speed
        attrs[5] = speed
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def _release_modem_control_lines(self):
        bits = 0
        bits |= getattr(termios, 'TIOCM_DTR', 0)
        bits |= getattr(termios, 'TIOCM_RTS', 0)
        if bits and hasattr(termios, 'TIOCMBIC'):
            try:
                fcntl.ioctl(self.fd, termios.TIOCMBIC, array.array('i', [bits]), True)
            except OSError:
                pass

    @staticmethod
    def _baud_constant(baudrate):
        name = f'B{int(baudrate)}'
        if not hasattr(termios, name):
            raise ValueError(f'Unsupported baudrate: {baudrate}')
        return getattr(termios, name)

    def read_lines(self) -> List[str]:
        lines = []
        while True:
            try:
                chunk = os.read(self.fd, 4096)
            except BlockingIOError:
                break
            if not chunk:
                break
            self.buffer.extend(chunk)

        while b'\n' in self.buffer:
            raw, _, rest = self.buffer.partition(b'\n')
            self.buffer = bytearray(rest)
            text = raw.decode('utf-8', errors='replace').strip()
            if text:
                lines.append(text)
        return lines

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class UwbSerialNode(Node):
    """Read UWB range lines from ESP32 USB serial ports and publish JSON."""

    def __init__(self):
        super().__init__('uwb_serial_node')

        self.declare_parameter('ports', 'auto')
        self.declare_parameter('exclude_ports', '')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('worker_id', 'worker1')
        self.declare_parameter('ranges_topic', '/uwb/ranges')
        self.declare_parameter('raw_topic', '/uwb/raw_lines')
        self.declare_parameter('poll_period_sec', 0.02)

        self.worker_id = self.get_parameter('worker_id').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.max_distance_m = float(
            self.declare_parameter('max_distance_m', 60.0).value
        )
        self.last_retry_time = 0.0
        self.last_invalid_log_time = 0.0
        self.ranges_pub = self.create_publisher(
            String,
            self.get_parameter('ranges_topic').value,
            10,
        )
        self.raw_pub = self.create_publisher(
            String,
            self.get_parameter('raw_topic').value,
            10,
        )

        self.readers: List[SerialLineReader] = []
        self.pending_ports: List[str] = []
        for port in self._resolve_ports():
            if not self._open_port(port):
                self.pending_ports.append(port)

        period = float(self.get_parameter('poll_period_sec').value)
        self.timer = self.create_timer(max(0.005, period), self.tick)
        self.get_logger().info(
            'UWB serial reader active on '
            f'{[reader.path for reader in self.readers]}'
        )

    def _resolve_ports(self):
        ports_value = self.get_parameter('ports').value
        exclude = set(self._split_ports(self.get_parameter('exclude_ports').value))
        if str(ports_value).strip().lower() != 'auto':
            return [
                port
                for port in self._split_ports(ports_value)
                if port and port not in exclude
            ]

        by_id = sorted(glob.glob('/dev/serial/by-id/*'))
        if by_id:
            resolved = [os.path.realpath(path) for path in by_id]
        else:
            resolved = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
        return [port for port in resolved if port not in exclude]

    @staticmethod
    def _split_ports(value):
        return [item.strip() for item in str(value).split(',') if item.strip()]

    def _open_port(self, port):
        try:
            self.readers.append(SerialLineReader(port, self.baudrate))
            self.get_logger().info(f'Opened UWB serial port {port}')
            return True
        except Exception as exc:
            self.get_logger().warning(f'Could not open {port}: {exc}')
            return False

    def _retry_pending_ports(self):
        if not self.pending_ports:
            return

        now = time.time()
        if now - self.last_retry_time < 1.0:
            return
        self.last_retry_time = now

        remaining = []
        open_paths = {reader.path for reader in self.readers}
        for port in self.pending_ports:
            if port in open_paths:
                continue
            if self._open_port(port):
                continue
            remaining.append(port)
        self.pending_ports = remaining

    def tick(self):
        self._retry_pending_ports()
        if not self.readers:
            return

        fds = [reader.fd for reader in self.readers if reader.fd is not None]
        if not fds:
            return
        ready, _, _ = select.select(fds, [], [], 0)
        if not ready:
            return

        by_fd = {reader.fd: reader for reader in self.readers}
        for fd in ready:
            reader = by_fd[fd]
            try:
                lines = reader.read_lines()
            except OSError as exc:
                self.get_logger().warning(f'Read failed on {reader.path}: {exc}')
                reader.close()
                self.readers.remove(reader)
                if reader.path not in self.pending_ports:
                    self.pending_ports.append(reader.path)
                continue

            for line in lines:
                self._publish_raw(reader.path, line)
                for sample in self._parse_line(reader.path, line):
                    self._publish_range(sample)

    def _publish_raw(self, port, line):
        msg = String()
        msg.data = json.dumps({
            'port': port,
            'line': line,
            'time': time.time(),
        }, separators=(',', ':'))
        self.raw_pub.publish(msg)

    def _publish_range(self, sample):
        distance_m = float(sample.get('distance_m', 0.0))
        if distance_m <= 0.0 or distance_m > self.max_distance_m:
            now = time.time()
            if now - self.last_invalid_log_time > 2.0:
                self.get_logger().warning(
                    'Ignoring invalid UWB distance '
                    f'{distance_m:.3f} m from anchor {sample.get("anchor_id")}'
                )
                self.last_invalid_log_time = now
            return

        msg = String()
        msg.data = json.dumps(sample, separators=(',', ':'))
        self.ranges_pub.publish(msg)

    def _parse_line(self, port, line) -> Iterable[Dict]:
        parsed_json = self._parse_json(port, line)
        if parsed_json:
            return parsed_json

        matches = list(DISTANCES_RE.finditer(line))
        if not matches:
            matches = list(KEY_VALUE_RE.finditer(line))

        samples = []
        for match in matches:
            value = float(match.group('value'))
            unit = (match.groupdict().get('unit') or 'cm').lower()
            distance_m = value / 100.0 if unit == 'cm' else value
            samples.append(self._sample(
                port=port,
                worker_id=self.worker_id,
                anchor_id=int(match.group('anchor')),
                distance_m=distance_m,
                source_line=line,
            ))
        return samples

    def _parse_json(self, port, line) -> Optional[List[Dict]]:
        if not line.startswith('{'):
            return None
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            return None

        worker_id = str(data.get('worker_id', self.worker_id))
        if 'distances' in data and isinstance(data['distances'], dict):
            samples = []
            for anchor_id, distance in data['distances'].items():
                samples.append(self._sample(
                    port=port,
                    worker_id=worker_id,
                    anchor_id=int(anchor_id),
                    distance_m=float(distance),
                    source_line=line,
                ))
            return samples

        if 'anchor_id' in data and 'distance_m' in data:
            return [self._sample(
                port=port,
                worker_id=worker_id,
                anchor_id=int(data['anchor_id']),
                distance_m=float(data['distance_m']),
                source_line=line,
            )]
        return None

    @staticmethod
    def _sample(port, worker_id, anchor_id, distance_m, source_line):
        return {
            'time': time.time(),
            'port': port,
            'worker_id': worker_id,
            'anchor_id': anchor_id,
            'distance_m': distance_m,
            'source_line': source_line,
        }

    def destroy_node(self):
        for reader in self.readers:
            reader.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UwbSerialNode()
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
            node.tick()
            rclpy.spin_once(node, timeout_sec=0.02)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
