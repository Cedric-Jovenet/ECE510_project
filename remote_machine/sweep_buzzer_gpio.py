#!/usr/bin/env python3
import os
import time

GPIO_BASE = int(os.environ.get('GPIO_BASE', '571'))
BUZZER_GPIO = int(os.environ.get('BUZZER_GPIO', '19'))
SYSFS_GPIO = GPIO_BASE + BUZZER_GPIO
GPIO_PATH = f'/sys/class/gpio/gpio{SYSFS_GPIO}'

FREQUENCIES = (500, 800, 1000, 1200, 1500, 1800, 2200, 2700, 3300, 4000)


def write_file(path, value):
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(str(value))


def wait(seconds):
    deadline = time.perf_counter() + seconds
    while time.perf_counter() < deadline:
        pass


def write_value(fd, value):
    os.lseek(fd, 0, os.SEEK_SET)
    os.write(fd, b'1' if value else b'0')


def tone(fd, frequency, duration):
    half = 0.5 / frequency
    deadline = time.perf_counter() + duration
    while time.perf_counter() < deadline:
        write_value(fd, True)
        wait(half)
        write_value(fd, False)
        wait(half)
    write_value(fd, False)


def main():
    if not os.path.isdir(GPIO_PATH):
        write_file('/sys/class/gpio/export', SYSFS_GPIO)
        time.sleep(0.05)
    write_file(os.path.join(GPIO_PATH, 'direction'), 'out')
    fd = os.open(os.path.join(GPIO_PATH, 'value'), os.O_WRONLY)
    try:
        print('Buzzer frequency sweep on GPIO19')
        for frequency in FREQUENCIES:
            print(f'{frequency} Hz')
            for _ in range(2):
                tone(fd, frequency, 0.22)
                time.sleep(0.18)
            time.sleep(0.65)
        print('Sweep complete')
    finally:
        write_value(fd, False)
        os.close(fd)


if __name__ == '__main__':
    main()
