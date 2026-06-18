#!/usr/bin/env bash
set -euo pipefail

pkill -f iot_supervisor_node 2>/dev/null || true
pkill -f 'ros2 run securite_fusion iot_supervisor_node' 2>/dev/null || true
sleep 1

sudo dtoverlay pwm pin=19 func=2 || true

PWM_CHIP="${PWM_CHIP:-/sys/class/pwm/pwmchip0}"
PWM_CHANNEL="${PWM_CHANNEL:-1}"
PWM_PATH="$PWM_CHIP/pwm$PWM_CHANNEL"

if [[ ! -d "$PWM_PATH" ]]; then
  echo "$PWM_CHANNEL" | sudo tee "$PWM_CHIP/export" >/dev/null || true
  sleep 0.2
fi

for frequency in 700 1000 1400; do
  period_ns=$((1000000000 / frequency))
  duty_ns=$((period_ns / 2))
  echo 0 | sudo tee "$PWM_PATH/enable" >/dev/null 2>&1 || true
  echo "$period_ns" | sudo tee "$PWM_PATH/period" >/dev/null
  echo "$duty_ns" | sudo tee "$PWM_PATH/duty_cycle" >/dev/null
  echo 1 | sudo tee "$PWM_PATH/enable" >/dev/null
  sleep 0.18
  echo 0 | sudo tee "$PWM_PATH/enable" >/dev/null
  sleep 0.35
done
