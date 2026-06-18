#!/usr/bin/env python3
import os
import time

GPIO_BASE = int(os.environ.get('GPIO_BASE', '571'))
BUZZER_GPIO = int(os.environ.get('BUZZER_GPIO', '19'))
SYSFS_GPIO = GPIO_BASE + BUZZER_GPIO
GPIO_PATH = f'/sys/class/gpio/gpio{SYSFS_GPIO}'


def write_file(path, value):
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(str(value))


def wait(seconds):
    deadline = time.perf_counter() + seconds
    while time.perf_counter() < deadline:
        pass


def tone(fd, frequency, duration):
    half = 0.5 / frequency
    deadline = time.perf_counter() + duration
    while time.perf_counter() < deadline:
        os.lseek(fd, 0, os.SEEK_SET)
        os.write(fd, b'1')
        wait(half)
        os.lseek(fd, 0, os.SEEK_SET)
        os.write(fd, b'0')
        wait(half)
    os.lseek(fd, 0, os.SEEK_SET)
    os.write(fd, b'0')


def main():
    if not os.path.isdir(GPIO_PATH):
        write_file('/sys/class/gpio/export', SYSFS_GPIO)
        time.sleep(0.05)
    write_file(os.path.join(GPIO_PATH, 'direction'), 'out')
    fd = os.open(os.path.join(GPIO_PATH, 'value'), os.O_WRONLY)
    try:
        for frequency in (500, 700, 1000):
            print(f'tone {frequency} Hz')
            tone(fd, frequency, 0.25)
            time.sleep(0.45)
    finally:
        os.lseek(fd, 0, os.SEEK_SET)
        os.write(fd, b'0')
        os.close(fd)


if __name__ == '__main__':
    main()
