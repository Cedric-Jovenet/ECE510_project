#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
PORT="${PORT:-8090}"
LOG="${LOG:-/tmp/iot_base_station.log}"

pkill -f 'iot_base_station.py' 2>/dev/null || true
for _ in $(seq 1 25); do
  if ! ss -ltn "sport = :$PORT" 2>/dev/null | grep -q ":$PORT"; then
    break
  fi
  sleep 0.2
done
nohup python3 iot_base_station.py --port "$PORT" > "$LOG" 2>&1 &
sleep 1
echo "Dashboard: http://$(hostname -I | awk '{print $1}'):$PORT/"
echo "Logs: tail -f $LOG"
