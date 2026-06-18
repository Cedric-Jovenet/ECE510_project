# Deployment

These commands assume the current lab network addresses.

## Machine Raspberry Pi

Copy the updated ROS 2 package files to the machine Pi, then rebuild and restart:

```powershell
pscp remote_machine\src\securite_fusion\securite_fusion\iot_supervisor_node.py `
  ece510@10.152.83.176:/home/ece510/ece510/src/securite_fusion/securite_fusion/iot_supervisor_node.py

pscp remote_machine\start_securite_fusion.sh `
  ece510@10.152.83.176:/home/ece510/ece510/start_securite_fusion.sh
```

```bash
cd /home/ece510/ece510
chmod +x start_securite_fusion.sh
. /opt/ros/jazzy/setup.bash
colcon build --packages-select securite_fusion
./start_securite_fusion.sh restart
```

The supervisor publishes accident reports after the post-accident evidence
window has elapsed. By default this window is 5 seconds before and 5 seconds
after the accident.

Useful environment overrides:

```bash
IOT_EVIDENCE_BEFORE_SEC=5.0
IOT_EVIDENCE_AFTER_SEC=5.0
IOT_UWB_HISTORY_PERIOD_SEC=0.25
IOT_VIDEO_CLIP_BEFORE_SEC=5.0
IOT_VIDEO_CLIP_AFTER_SEC=5.0
./start_securite_fusion.sh restart
```

## Base Station Raspberry Pi

Copy the dashboard server and restart:

```powershell
pscp base_station\iot_base_station.py `
  carpe@10.152.83.65:/home/carpe/iot_base_station/iot_base_station.py
```

```bash
cd /home/carpe/iot_base_station
python3 -m py_compile iot_base_station.py
./start_iot_base_station.sh
```

## ESP32 Worker Tag

Create the local, untracked credential file:

```text
esp32_worker_tag/include/config_local.h
```

using `esp32_worker_tag/include/config_local.example.h` as the template.

Build and upload with PlatformIO:

```powershell
platformio run -d esp32_worker_tag
platformio run -d esp32_worker_tag -t upload
```

## Verification

From the Windows workstation:

```powershell
Invoke-WebRequest -UseBasicParsing http://10.152.83.65:8090/
Invoke-RestMethod http://10.152.83.65:8090/api/state
Invoke-WebRequest -UseBasicParsing http://10.152.83.176:8080/snapshot.jpg
```

Expected dashboard behavior:

- System status shows current device pings.
- Accident reports are collapsible.
- Video and UWB maps appear only inside accident reports.
- Accident videos use archived clips served by `recorded_clip.mjpg`; dynamic
  `/clip.mjpg` URLs are not treated as report evidence.
- The dashboard video evidence view uses recorded frames through
  `recorded_frame.jpg`, with local play/pause and frame scrubbing controls.
- UWB maps show the recorded movement window around the accident, not a global
  live map.

Machine video clips are archived under `/tmp/securite_fusion_clips` on the
machine Raspberry Pi.
