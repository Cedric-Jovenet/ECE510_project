#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${WORKSPACE:-$HOME/ece510}"
LOG_DIR="${LOG_DIR:-/tmp}"

as_double() {
  if [[ "$1" =~ ^-?[0-9]+$ ]]; then
    printf '%s.0' "$1"
  else
    printf '%s' "$1"
  fi
}

CAMERA_SOURCE="${CAMERA_SOURCE:-csi}"
CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video8}"
CSI_CAMERA_DEVICE="${CSI_CAMERA_DEVICE:-auto}"
CSI_MEDIA_DEVICE="${CSI_MEDIA_DEVICE:-auto}"
CSI_BAYER_PATTERN="${CSI_BAYER_PATTERN:-RG}"
CSI_ANALOGUE_GAIN="${CSI_ANALOGUE_GAIN:-120}"
CSI_DIGITAL_GAIN="${CSI_DIGITAL_GAIN:-1024}"
CSI_EXPOSURE="${CSI_EXPOSURE:-1600}"
CSI_LEVEL_HIGH_PERCENTILE="${CSI_LEVEL_HIGH_PERCENTILE:-90.0}"
CSI_GAMMA="${CSI_GAMMA:-0.55}"
LIDAR_PORT="${LIDAR_PORT:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}"
WEB_PORT="${WEB_PORT:-8080}"
GPIO_BASE="${GPIO_BASE:-571}"
BUZZER_GPIO="${BUZZER_GPIO:-19}"
BUZZER_DRIVER="${BUZZER_DRIVER:-gpio}"
BUZZER_PWM_CHIP="${BUZZER_PWM_CHIP:-/sys/class/pwm/pwmchip0}"
BUZZER_PWM_CHANNEL="${BUZZER_PWM_CHANNEL:-1}"
BUZZER_WARNING_TONE_HZ="${BUZZER_WARNING_TONE_HZ:-1800.0}"
BUZZER_CRITICAL_TONE_HZ="${BUZZER_CRITICAL_TONE_HZ:-2200.0}"
BUZZER_WARNING_PERIOD_SEC="${BUZZER_WARNING_PERIOD_SEC:-1.0}"
BUZZER_CRITICAL_PERIOD_SEC="${BUZZER_CRITICAL_PERIOD_SEC:-0.32}"
BUZZER_DURATION_SEC="${BUZZER_DURATION_SEC:-0.22}"
IOT_BASE_STATION_URL="${IOT_BASE_STATION_URL:-http://10.152.83.65:8090}"
IOT_VIEWER_BASE_URL="${IOT_VIEWER_BASE_URL:-http://10.152.83.176:8080}"
IOT_LORA_GATEWAY_UDP_HOST="${IOT_LORA_GATEWAY_UDP_HOST:-10.152.83.255}"
IOT_LORA_GATEWAY_UDP_PORT="${IOT_LORA_GATEWAY_UDP_PORT:-8895}"
IOT_REPORT_COOLDOWN_SEC="${IOT_REPORT_COOLDOWN_SEC:-60.0}"
IOT_VIDEO_CLIP_BEFORE_SEC="${IOT_VIDEO_CLIP_BEFORE_SEC:-5.0}"
IOT_VIDEO_CLIP_AFTER_SEC="${IOT_VIDEO_CLIP_AFTER_SEC:-5.0}"
IOT_VIDEO_CLIP_FPS="${IOT_VIDEO_CLIP_FPS:-4.0}"
IOT_EVIDENCE_BEFORE_SEC="${IOT_EVIDENCE_BEFORE_SEC:-$IOT_VIDEO_CLIP_BEFORE_SEC}"
IOT_EVIDENCE_AFTER_SEC="${IOT_EVIDENCE_AFTER_SEC:-$IOT_VIDEO_CLIP_AFTER_SEC}"
IOT_UWB_HISTORY_PERIOD_SEC="${IOT_UWB_HISTORY_PERIOD_SEC:-0.25}"

if [[ -z "${CAMERA_FOV_DEG:-}" ]]; then
  if [[ "$CAMERA_SOURCE" == "csi" ]]; then
    CAMERA_FOV_DEG="62.2"
  else
    CAMERA_FOV_DEG="78.0"
  fi
fi
CAMERA_YAW_OFFSET_DEG="${CAMERA_YAW_OFFSET_DEG:-0.0}"
LIDAR_FRONT_ANGLE_DEG="${LIDAR_FRONT_ANGLE_DEG:-0.0}"
LIDAR_CONE_HALF_ANGLE_DEG="${LIDAR_CONE_HALF_ANGLE_DEG:-30.0}"
BBOX_MARGIN_PX="${BBOX_MARGIN_PX:-6.0}"
PERSON_DISTANCE_PERCENTILE="${PERSON_DISTANCE_PERCENTILE:-20.0}"

YOLO_CONFIDENCE="${YOLO_CONFIDENCE:-0.25}"
YOLO_MAX_FPS="${YOLO_MAX_FPS:-1.0}"
FUSION_SAFETY_THRESHOLD_M="${FUSION_SAFETY_THRESHOLD_M:-2.0}"
UWB_SOURCE="${UWB_SOURCE:-serial}"
UWB_PORTS="${UWB_PORTS:-cp2104}"
UWB_EXCLUDE_PORTS="${UWB_EXCLUDE_PORTS:-}"
UWB_EXPECTED_PORT_COUNT="${UWB_EXPECTED_PORT_COUNT:-3}"
UWB_TAG_UDP_PORT="${UWB_TAG_UDP_PORT:-8890}"
UWB_ANCHORS_JSON="${UWB_ANCHORS_JSON:-{\"1\":[-0.35,0.0],\"2\":[0.35,0.0],\"3\":[0.0,0.55]}}"
UWB_CRITICAL_RADIUS_M="${UWB_CRITICAL_RADIUS_M:-2.0}"
UWB_WARNING_RADIUS_M="${UWB_WARNING_RADIUS_M:-2.0}"
ULTRASONIC_CRITICAL_DISTANCE_M="${ULTRASONIC_CRITICAL_DISTANCE_M:-1.0}"
ULTRASONIC_WARNING_DISTANCE_M="${ULTRASONIC_WARNING_DISTANCE_M:-1.0}"
ENABLE_RPLIDAR="${ENABLE_RPLIDAR:-1}"
ENABLE_CAMERA="${ENABLE_CAMERA:-1}"
ENABLE_YOLO="${ENABLE_YOLO:-1}"
ENABLE_FUSION="${ENABLE_FUSION:-1}"
ENABLE_ULTRASONIC="${ENABLE_ULTRASONIC:-1}"
ENABLE_IOT_SUPERVISOR="${ENABLE_IOT_SUPERVISOR:-1}"

CAMERA_FOV_DEG="$(as_double "$CAMERA_FOV_DEG")"
CAMERA_YAW_OFFSET_DEG="$(as_double "$CAMERA_YAW_OFFSET_DEG")"
LIDAR_FRONT_ANGLE_DEG="$(as_double "$LIDAR_FRONT_ANGLE_DEG")"
LIDAR_CONE_HALF_ANGLE_DEG="$(as_double "$LIDAR_CONE_HALF_ANGLE_DEG")"
BBOX_MARGIN_PX="$(as_double "$BBOX_MARGIN_PX")"
PERSON_DISTANCE_PERCENTILE="$(as_double "$PERSON_DISTANCE_PERCENTILE")"
CSI_LEVEL_HIGH_PERCENTILE="$(as_double "$CSI_LEVEL_HIGH_PERCENTILE")"
CSI_GAMMA="$(as_double "$CSI_GAMMA")"
YOLO_CONFIDENCE="$(as_double "$YOLO_CONFIDENCE")"
YOLO_MAX_FPS="$(as_double "$YOLO_MAX_FPS")"
FUSION_SAFETY_THRESHOLD_M="$(as_double "$FUSION_SAFETY_THRESHOLD_M")"
UWB_CRITICAL_RADIUS_M="$(as_double "$UWB_CRITICAL_RADIUS_M")"
UWB_WARNING_RADIUS_M="$(as_double "$UWB_WARNING_RADIUS_M")"
ULTRASONIC_CRITICAL_DISTANCE_M="$(as_double "$ULTRASONIC_CRITICAL_DISTANCE_M")"
ULTRASONIC_WARNING_DISTANCE_M="$(as_double "$ULTRASONIC_WARNING_DISTANCE_M")"
IOT_REPORT_COOLDOWN_SEC="$(as_double "$IOT_REPORT_COOLDOWN_SEC")"
IOT_VIDEO_CLIP_BEFORE_SEC="$(as_double "$IOT_VIDEO_CLIP_BEFORE_SEC")"
IOT_VIDEO_CLIP_AFTER_SEC="$(as_double "$IOT_VIDEO_CLIP_AFTER_SEC")"
IOT_VIDEO_CLIP_FPS="$(as_double "$IOT_VIDEO_CLIP_FPS")"
IOT_EVIDENCE_BEFORE_SEC="$(as_double "$IOT_EVIDENCE_BEFORE_SEC")"
IOT_EVIDENCE_AFTER_SEC="$(as_double "$IOT_EVIDENCE_AFTER_SEC")"
IOT_UWB_HISTORY_PERIOD_SEC="$(as_double "$IOT_UWB_HISTORY_PERIOD_SEC")"
BUZZER_WARNING_TONE_HZ="$(as_double "$BUZZER_WARNING_TONE_HZ")"
BUZZER_CRITICAL_TONE_HZ="$(as_double "$BUZZER_CRITICAL_TONE_HZ")"
BUZZER_WARNING_PERIOD_SEC="$(as_double "$BUZZER_WARNING_PERIOD_SEC")"
BUZZER_CRITICAL_PERIOD_SEC="$(as_double "$BUZZER_CRITICAL_PERIOD_SEC")"
BUZZER_DURATION_SEC="$(as_double "$BUZZER_DURATION_SEC")"

ACTION="${1:-restart}"

cd "$WORKSPACE"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
source /opt/ros/jazzy/setup.bash
if [[ ! -f install/setup.bash ]]; then
  echo "[setup] install/setup.bash introuvable, build du workspace..."
  colcon build --packages-select rplidar_ros securite_fusion
fi
source install/setup.bash

prepare_gpio_access() {
  if [[ "$ENABLE_ULTRASONIC" != "1" && "$ENABLE_IOT_SUPERVISOR" != "1" ]]; then
    return
  fi

  local pins=()
  if [[ "$ENABLE_ULTRASONIC" == "1" ]]; then
    pins+=(26 25 21 20)
  fi
  if [[ "$ENABLE_IOT_SUPERVISOR" == "1" ]]; then
    if [[ "$BUZZER_DRIVER" != "pwm" ]]; then
      pins+=("$BUZZER_GPIO")
    fi
    if [[ "$BUZZER_DRIVER" == "auto" || "$BUZZER_DRIVER" == "pwm" ]]; then
      sudo -n dtoverlay pwm pin="$BUZZER_GPIO" func=2 2>/dev/null || true
      local pwm_path="$BUZZER_PWM_CHIP/pwm$BUZZER_PWM_CHANNEL"
      if [[ ! -d "$pwm_path" && -w "$BUZZER_PWM_CHIP/export" ]]; then
        echo "$BUZZER_PWM_CHANNEL" > "$BUZZER_PWM_CHIP/export" 2>/dev/null || true
        sleep 0.05
      elif [[ ! -d "$pwm_path" ]]; then
        sudo -n sh -c "echo $BUZZER_PWM_CHANNEL > '$BUZZER_PWM_CHIP/export'" 2>/dev/null || true
        sleep 0.05
      fi
      if [[ -d "$pwm_path" && ! -w "$pwm_path/enable" ]]; then
        sudo -n chown "$USER:$USER" \
          "$pwm_path/enable" "$pwm_path/period" "$pwm_path/duty_cycle" 2>/dev/null || true
      fi
    fi
  fi

  local bcm gpio path
  for bcm in "${pins[@]}"; do
    gpio=$((GPIO_BASE + bcm))
    path="/sys/class/gpio/gpio$gpio"
    if [[ ! -d "$path" ]]; then
      if [[ -w /sys/class/gpio/export ]]; then
        echo "$gpio" > /sys/class/gpio/export 2>/dev/null || true
      else
        sudo -n sh -c "echo $gpio > /sys/class/gpio/export" 2>/dev/null || true
      fi
      sleep 0.05
    fi
    if [[ -e "$path/direction" && ! -w "$path/direction" ]]; then
      sudo -n chown "$USER:$USER" "$path/direction" "$path/value" 2>/dev/null || true
    fi
  done
}

detect_csi_media_device() {
  local candidate
  for candidate in /dev/media*; do
    [[ -e "$candidate" ]] || continue
    if media-ctl -d "$candidate" -p 2>/dev/null | grep -q '^model[[:space:]]*rp1-cfe'; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  return 1
}

stop_nodes() {
  echo "[stop] Arret des anciens noeuds securite_fusion/RPLidar..."
  pkill -f '/securite_fusion/iot_supervisor_node' 2>/dev/null || true
  pkill -f 'ros2 run securite_fusion iot_supervisor_node' 2>/dev/null || true
  pkill -f '/securite_fusion/ultrasonic_node' 2>/dev/null || true
  pkill -f 'ros2 run securite_fusion ultrasonic_node' 2>/dev/null || true
  pkill -f '/securite_fusion/web_viewer_node' 2>/dev/null || true
  pkill -f 'ros2 run securite_fusion web_viewer_node' 2>/dev/null || true
  pkill -f '/securite_fusion/fusion_node' 2>/dev/null || true
  pkill -f 'ros2 run securite_fusion fusion_node' 2>/dev/null || true
  pkill -f '/securite_fusion/person_detector_node' 2>/dev/null || true
  pkill -f 'ros2 run securite_fusion person_detector_node' 2>/dev/null || true
  pkill -f '/securite_fusion/usb_camera_node' 2>/dev/null || true
  pkill -f 'ros2 run securite_fusion usb_camera_node' 2>/dev/null || true
  pkill -f '/securite_fusion/camera_raw_node' 2>/dev/null || true
  pkill -f 'ros2 run securite_fusion camera_raw_node' 2>/dev/null || true
  pkill -f '/securite_fusion/uwb_serial_node' 2>/dev/null || true
  pkill -f 'ros2 run securite_fusion uwb_serial_node' 2>/dev/null || true
  pkill -f '/securite_fusion/uwb_udp_node' 2>/dev/null || true
  pkill -f 'ros2 run securite_fusion uwb_udp_node' 2>/dev/null || true
  pkill -f '/securite_fusion/uwb_position_node' 2>/dev/null || true
  pkill -f 'ros2 run securite_fusion uwb_position_node' 2>/dev/null || true
  pkill -f '/securite_fusion/uwb_simulator_node' 2>/dev/null || true
  pkill -f 'ros2 run securite_fusion uwb_simulator_node' 2>/dev/null || true
  pkill -f '/rplidar_ros/rplidar_node' 2>/dev/null || true
  pkill -f 'ros2 launch rplidar_ros rplidar_a1_launch.py' 2>/dev/null || true
  sleep 1
}

start_nodes() {
  rm -f \
    "$LOG_DIR/rplidar_a1.log" \
    "$LOG_DIR/usb_camera_node.log" \
    "$LOG_DIR/camera_raw_node.log" \
    "$LOG_DIR/person_detector_node.log" \
    "$LOG_DIR/fusion_node.log" \
    "$LOG_DIR/web_viewer_node.log" \
    "$LOG_DIR/uwb_serial_node.log" \
    "$LOG_DIR/uwb_udp_node.log" \
    "$LOG_DIR/uwb_position_node.log" \
    "$LOG_DIR/uwb_simulator_node.log" \
    "$LOG_DIR/ultrasonic_node.log" \
    "$LOG_DIR/iot_supervisor_node.log"

  prepare_gpio_access

  if [[ "$ENABLE_RPLIDAR" == "1" ]]; then
    echo "[start] RPLidar A1 sur $LIDAR_PORT"
    nohup ros2 launch rplidar_ros rplidar_a1_launch.py \
      serial_port:="$LIDAR_PORT" \
      > "$LOG_DIR/rplidar_a1.log" 2>&1 &
  fi

  if [[ "$ENABLE_CAMERA" == "1" ]]; then
    if [[ "$CAMERA_SOURCE" == "csi" ]]; then
      if [[ "$CSI_MEDIA_DEVICE" == "auto" ]]; then
        detected_media_device="$(detect_csi_media_device || true)"
        if [[ -n "$detected_media_device" ]]; then
          echo "[start] Controleur CSI actuellement detecte sur $detected_media_device"
        else
          echo "[warn] Camera CSI absente au demarrage; attente automatique de son retour"
        fi
      fi
      echo "[start] Camera CSI sur $CSI_CAMERA_DEVICE via $CSI_MEDIA_DEVICE"
      nohup ros2 run securite_fusion camera_raw_node --ros-args \
        -p device:="$CSI_CAMERA_DEVICE" \
        -p media_device:="$CSI_MEDIA_DEVICE" \
        -p width:=640 \
        -p height:=480 \
        -p bayer_pattern:="$CSI_BAYER_PATTERN" \
        -p analogue_gain:="$CSI_ANALOGUE_GAIN" \
        -p digital_gain:="$CSI_DIGITAL_GAIN" \
        -p exposure:="$CSI_EXPOSURE" \
        -p level_high_percentile:="$CSI_LEVEL_HIGH_PERCENTILE" \
        -p gamma:="$CSI_GAMMA" \
        > "$LOG_DIR/camera_raw_node.log" 2>&1 &
    elif [[ "$CAMERA_SOURCE" == "usb" ]]; then
      echo "[start] Webcam USB sur $CAMERA_DEVICE"
      nohup ros2 run securite_fusion usb_camera_node --ros-args \
        -p device:="$CAMERA_DEVICE" \
        -p width:=640 \
        -p height:=480 \
        -p fps:=30.0 \
        > "$LOG_DIR/usb_camera_node.log" 2>&1 &
    else
      echo "CAMERA_SOURCE invalide: $CAMERA_SOURCE (utilise usb ou csi)" >&2
      exit 2
    fi
  fi

  if [[ "$ENABLE_YOLO" == "1" ]]; then
    echo "[start] Detection personne YOLO"
    nohup ros2 run securite_fusion person_detector_node --ros-args \
      -p confidence_threshold:="$YOLO_CONFIDENCE" \
      -p max_fps:="$YOLO_MAX_FPS" \
      > "$LOG_DIR/person_detector_node.log" 2>&1 &
  fi

  if [[ "$ENABLE_FUSION" == "1" ]]; then
    echo "[start] Fusion LiDAR + camera"
    nohup ros2 run securite_fusion fusion_node --ros-args \
      -p safety_threshold_m:="$FUSION_SAFETY_THRESHOLD_M" \
      -p camera_horizontal_fov_deg:="$CAMERA_FOV_DEG" \
      -p camera_yaw_offset_deg:="$CAMERA_YAW_OFFSET_DEG" \
      -p lidar_front_angle_deg:="$LIDAR_FRONT_ANGLE_DEG" \
      -p lidar_cone_half_angle_deg:="$LIDAR_CONE_HALF_ANGLE_DEG" \
      -p bbox_x_margin_px:="$BBOX_MARGIN_PX" \
      -p person_distance_percentile:="$PERSON_DISTANCE_PERCENTILE" \
      > "$LOG_DIR/fusion_node.log" 2>&1 &
  fi

  if [[ "$UWB_SOURCE" == "serial" ]]; then
    echo "[start] UWB depuis ESP32 USB ($UWB_PORTS)"
    uwb_serial_args=(-p ports:="$UWB_PORTS")
    if [[ -n "$UWB_EXCLUDE_PORTS" ]]; then
      uwb_serial_args+=(-p exclude_ports:="$UWB_EXCLUDE_PORTS")
    fi
    nohup ros2 run securite_fusion uwb_serial_node --ros-args \
      "${uwb_serial_args[@]}" \
      -p expected_port_count:="$UWB_EXPECTED_PORT_COUNT" \
      > "$LOG_DIR/uwb_serial_node.log" 2>&1 &
    nohup ros2 run securite_fusion uwb_udp_node --ros-args \
      -p port:="$UWB_TAG_UDP_PORT" \
      > "$LOG_DIR/uwb_udp_node.log" 2>&1 &
    nohup ros2 run securite_fusion uwb_position_node --ros-args \
      -p anchor_positions_json:="'$UWB_ANCHORS_JSON'" \
      -p critical_radius_m:="$UWB_CRITICAL_RADIUS_M" \
      -p warning_radius_m:="$UWB_WARNING_RADIUS_M" \
      > "$LOG_DIR/uwb_position_node.log" 2>&1 &
  elif [[ "$UWB_SOURCE" == "sim" ]]; then
    echo "[start] UWB simulateur"
    nohup ros2 run securite_fusion uwb_simulator_node --ros-args \
      -p anchor_positions_json:="'$UWB_ANCHORS_JSON'" \
      > "$LOG_DIR/uwb_simulator_node.log" 2>&1 &
    nohup ros2 run securite_fusion uwb_position_node --ros-args \
      -p anchor_positions_json:="'$UWB_ANCHORS_JSON'" \
      -p critical_radius_m:="$UWB_CRITICAL_RADIUS_M" \
      -p warning_radius_m:="$UWB_WARNING_RADIUS_M" \
      > "$LOG_DIR/uwb_position_node.log" 2>&1 &
  elif [[ "$UWB_SOURCE" != "off" ]]; then
    echo "UWB_SOURCE invalide: $UWB_SOURCE (utilise off, sim ou serial)" >&2
    exit 2
  fi

  if [[ "$ENABLE_ULTRASONIC" == "1" ]]; then
    echo "[start] Ultrasons HC-SR04"
    nohup ros2 run securite_fusion ultrasonic_node --ros-args \
      -p gpio_base:="$GPIO_BASE" \
      -p critical_distance_m:="$ULTRASONIC_CRITICAL_DISTANCE_M" \
      -p warning_distance_m:="$ULTRASONIC_WARNING_DISTANCE_M" \
      > "$LOG_DIR/ultrasonic_node.log" 2>&1 &
  fi

  if [[ "$ENABLE_IOT_SUPERVISOR" == "1" ]]; then
    echo "[start] Supervision IoT + buzzer + reporting base station"
    nohup ros2 run securite_fusion iot_supervisor_node --ros-args \
      -p base_station_url:="$IOT_BASE_STATION_URL" \
      -p viewer_base_url:="$IOT_VIEWER_BASE_URL" \
      -p lora_gateway_udp_host:="$IOT_LORA_GATEWAY_UDP_HOST" \
      -p lora_gateway_udp_port:="$IOT_LORA_GATEWAY_UDP_PORT" \
      -p report_cooldown_sec:="$IOT_REPORT_COOLDOWN_SEC" \
      -p video_clip_before_sec:="$IOT_VIDEO_CLIP_BEFORE_SEC" \
      -p video_clip_after_sec:="$IOT_VIDEO_CLIP_AFTER_SEC" \
      -p video_clip_fps:="$IOT_VIDEO_CLIP_FPS" \
      -p evidence_before_sec:="$IOT_EVIDENCE_BEFORE_SEC" \
      -p evidence_after_sec:="$IOT_EVIDENCE_AFTER_SEC" \
      -p uwb_history_period_sec:="$IOT_UWB_HISTORY_PERIOD_SEC" \
      -p buzzer_driver:="$BUZZER_DRIVER" \
      -p gpio_base:="$GPIO_BASE" \
      -p buzzer_gpio:="$BUZZER_GPIO" \
      -p pwm_chip:="$BUZZER_PWM_CHIP" \
      -p pwm_channel:="$BUZZER_PWM_CHANNEL" \
      -p warning_tone_hz:="$BUZZER_WARNING_TONE_HZ" \
      -p critical_tone_hz:="$BUZZER_CRITICAL_TONE_HZ" \
      -p warning_buzz_period_sec:="$BUZZER_WARNING_PERIOD_SEC" \
      -p critical_buzz_period_sec:="$BUZZER_CRITICAL_PERIOD_SEC" \
      -p buzz_duration_sec:="$BUZZER_DURATION_SEC" \
      > "$LOG_DIR/iot_supervisor_node.log" 2>&1 &
  fi

  echo "[start] Viewer web sur le port $WEB_PORT"
  nohup ros2 run securite_fusion web_viewer_node --ros-args \
    -p port:="$WEB_PORT" \
    > "$LOG_DIR/web_viewer_node.log" 2>&1 &

  sleep 5
}

show_status() {
  echo
  echo "[status] Processus actifs"
  ps -u "$USER" -o pid=,comm=,args= \
    | grep -E 'rplidar|usb_camera_node|camera_raw_node|person_detector|fusion_node|web_viewer_node|uwb_|ultrasonic_node|iot_supervisor_node' \
    | grep -v grep || true

  echo
  echo "[status] Camera source: $CAMERA_SOURCE, FOV: $CAMERA_FOV_DEG deg"
  echo "[status] UWB source: $UWB_SOURCE"
  echo
  echo "[status] Topics"
  ros2 topic info /scan 2>/dev/null | grep 'Publisher count' || true
  ros2 topic info /image_raw 2>/dev/null | grep 'Publisher count' || true
  ros2 topic info /person_detections 2>/dev/null | grep 'Publisher count' || true
  ros2 topic info /fusion_overlay 2>/dev/null | grep 'Publisher count' || true
  ros2 topic info /uwb/workers 2>/dev/null | grep 'Publisher count' || true
  ros2 topic info /ultrasonic/status 2>/dev/null | grep 'Publisher count' || true
  ros2 topic info /iot/status 2>/dev/null | grep 'Publisher count' || true

  echo
  echo "[status] Fusion"
  curl -s --max-time 2 "http://127.0.0.1:$WEB_PORT/status.json" || true
  echo
  echo
  echo "[status] UWB"
  curl -s --max-time 2 "http://127.0.0.1:$WEB_PORT/uwb.json" || true
  echo

  echo
  local_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  if [[ -n "$local_ip" ]]; then
    echo "[ok] Viewer Windows: http://$local_ip:$WEB_PORT/"
    echo "[ok] Base dashboard cible: $IOT_BASE_STATION_URL/"
  else
    echo "[ok] Viewer Windows: http://<ip-de-la-pi>:$WEB_PORT/"
    echo "[ok] Base dashboard cible: $IOT_BASE_STATION_URL/"
  fi
  echo "[logs] tail -f $LOG_DIR/fusion_node.log"
}

show_logs() {
  echo "[logs] Fusion node. Ctrl+C pour quitter."
  tail -f "$LOG_DIR/fusion_node.log"
}

case "$ACTION" in
  start)
    start_nodes
    show_status
    ;;
  restart)
    stop_nodes
    start_nodes
    show_status
    ;;
  stop)
    stop_nodes
    ;;
  status)
    show_status
    ;;
  logs)
    show_logs
    ;;
  *)
    echo "Usage: $0 [start|restart|stop|status|logs]" >&2
    exit 2
    ;;
esac
