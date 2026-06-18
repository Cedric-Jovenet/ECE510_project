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

GPIO permissions for the ultrasonic sensors and buzzer are prepared by a small
systemd service. Install or refresh it after copying `prepare_machine_gpio.sh`
and `ece510-gpio-prepare.service`:

```bash
cd /home/ece510/ece510
chmod +x prepare_machine_gpio.sh
sudo cp prepare_machine_gpio.sh /usr/local/sbin/ece510-prepare-gpio.sh
sudo chown root:root /usr/local/sbin/ece510-prepare-gpio.sh
sudo chmod 755 /usr/local/sbin/ece510-prepare-gpio.sh
sudo cp ece510-gpio-prepare.service /etc/systemd/system/ece510-gpio-prepare.service
sudo systemctl daemon-reload
sudo systemctl enable --now ece510-gpio-prepare.service
```

The supervisor publishes accident reports after the post-accident evidence
window has elapsed. By default this window is 5 seconds before and 5 seconds
after the accident.

Useful environment overrides:

```bash
UWB_PORTS=cp2104
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

## ESP32 UWB Nodes

The UWB firmware is kept in `esp32_uwb_nodes/` so it is versioned with the rest
of the project. The current bring-up configuration is:

- `node1`: machine-side USB anchor/responder.
- `node2`: machine-side USB anchor/responder.
- `node3`: machine-side USB anchor/responder.
- `node4`: worker/tag initiator. This is the only UWB ESP32 that should move
  with the worker/tag.

Build the four binaries from Windows:

```powershell
platformio run -d esp32_uwb_nodes -e node1 -e node2 -e node3 -e node4
```

When flashing from the machine Raspberry Pi, identify the CP2104 adapters first:

```bash
ls -l /dev/serial/by-id/*CP2104*
```

Flash the three machine-side USB ESP32s with `node1`, `node2`, and `node3`.
Flash the worker/tag ESP32 with `node4`. After reboot, each ESP32 prints JSON
status lines such as
`{"type":"uwb_status","node_id":1,"role":"responder","num_nodes":4,...}`.
The anchors also publish successful tag range samples as
`{"type":"uwb_distance","worker_id":"worker4","anchor_id":1,...}` on their USB
serial ports; those lines are intentionally parseable by `uwb_serial_node.py`
and should also appear on `/uwb/raw_lines`.

Current lab flash commands:

```powershell
# Build locally on Windows
& C:\Users\cedri\.platformio\penv\Scripts\platformio.exe run -d esp32_uwb_nodes -e node1 -e node2 -e node3 -e node4

# Worker/tag UWB connected to the Windows PC on COM7
& C:\Users\cedri\.platformio\penv\Scripts\platformio.exe run -d esp32_uwb_nodes -e node4 -t upload --upload-port COM7
```

```bash
# Machine Raspberry Pi: stop ROS before flashing USB anchors
cd /home/ece510/ece510
./start_securite_fusion.sh stop

# Flash visible machine-side anchors
cd /home/ece510/uwb_flash/node1
~/esptool-venv/bin/esptool --chip esp32 --port /dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_023B6F01-if00-port0 --baud 460800 --before default-reset --after hard-reset write-flash --flash-mode dio --flash-freq 40m --flash-size detect 0x1000 bootloader.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware.bin

cd /home/ece510/uwb_flash/node2
~/esptool-venv/bin/esptool --chip esp32 --port /dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_023BB3CC-if00-port0 --baud 460800 --before default-reset --after hard-reset write-flash --flash-mode dio --flash-freq 40m --flash-size detect 0x1000 bootloader.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware.bin

# Use the same command for node3 after identifying the third CP2104 path:
ls -l /dev/serial/by-id/*CP2104*
cd /home/ece510/uwb_flash/node3
~/esptool-venv/bin/esptool --chip esp32 --port /dev/serial/by-id/<third-CP2104-id> --baud 460800 --before default-reset --after hard-reset write-flash --flash-mode dio --flash-freq 40m --flash-size detect 0x1000 bootloader.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware.bin

cd /home/ece510/ece510
./start_securite_fusion.sh restart
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
