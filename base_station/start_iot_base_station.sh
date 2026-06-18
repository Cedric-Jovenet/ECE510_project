#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
PORT="${PORT:-8090}"
LOG="${LOG:-/tmp/iot_base_station.log}"

pkill -f 'iot_base_station.py' 2>/dev/null || true
nohup python3 iot_base_station.py --port "$PORT" > "$LOG" 2>&1 &
sleep 1
echo "Dashboard: http://$(hostname -I | awk '{print $1}'):$PORT/"
echo "Logs: tail -f $LOG"

