#!/usr/bin/env bash
set -euo pipefail

GPIO_BASE="${GPIO_BASE:-571}"
TARGET_USER="${TARGET_USER:-ece510}"
TARGET_GROUP="${TARGET_GROUP:-ece510}"
BUZZER_GPIO="${BUZZER_GPIO:-19}"
BUZZER_PWM_CHIP="${BUZZER_PWM_CHIP:-/sys/class/pwm/pwmchip0}"
BUZZER_PWM_CHANNEL="${BUZZER_PWM_CHANNEL:-1}"
PINS=(26 25 21 20 19)

for bcm in "${PINS[@]}"; do
  gpio=$((GPIO_BASE + bcm))
  path="/sys/class/gpio/gpio${gpio}"
  if [[ ! -d "$path" ]]; then
    echo "$gpio" > /sys/class/gpio/export
    sleep 0.05
  fi
  if [[ -e "$path/direction" ]]; then
    chown "$TARGET_USER:$TARGET_GROUP" "$path/direction"
  fi
  if [[ -e "$path/value" ]]; then
    chown "$TARGET_USER:$TARGET_GROUP" "$path/value"
  fi
done

dtoverlay pwm pin="$BUZZER_GPIO" func=2 2>/dev/null || true

pwm_path="$BUZZER_PWM_CHIP/pwm$BUZZER_PWM_CHANNEL"
if [[ ! -d "$pwm_path" ]]; then
  echo "$BUZZER_PWM_CHANNEL" > "$BUZZER_PWM_CHIP/export" 2>/dev/null || true
  sleep 0.1
fi

if [[ -d "$pwm_path" ]]; then
  for file in enable period duty_cycle; do
    if [[ -e "$pwm_path/$file" ]]; then
      chown "$TARGET_USER:$TARGET_GROUP" "$pwm_path/$file"
    fi
  done
fi
