# ECE510 Safety IoT Project

Code for the ECE510 safety demo across the machine, worker tag, and LoRa base
station devices.

## Devices

- `remote_machine/`: Raspberry Pi machine stack with ROS 2, camera, LiDAR, UWB,
  ultrasonic sensors, buzzer, local demo viewer, and IoT accident reporting.
- `base_station/`: Raspberry Pi base station dashboard and LoRa receiver. It
  receives pings and accident reports from machines and worker tags.
- `esp32_worker_tag/`: ESP32 worker tag firmware with LoRa, GPS, IMU, and WiFi
  fallback.
- `esp32_iot_probe/`: ESP32 hardware probe firmware used for bring-up tests.
- `esp32_uwb_nodes/`: ESP32 DW3000 UWB firmware. In the current lab setup,
  `node1` is the machine-side initiator and `node2`/`node3` are worker/tag
  responders.

## Runtime Endpoints

- Machine demo viewer: `http://10.152.83.176:8080/`
- Base station manager dashboard: `http://10.152.83.65:8090/`

The dashboard has two separate roles:

- Live device status from machine and worker pings.
- Accident reports with recorded evidence around the event time, including the
  machine video clip and UWB movement window.

## Configuration

Do not commit real WiFi passwords or session passwords.

For the ESP32 worker tag, copy:

```text
esp32_worker_tag/include/config_local.example.h
```

to:

```text
esp32_worker_tag/include/config_local.h
```

then edit the local copy with the real lab WiFi values. `config_local.h` is
ignored by Git.

## Deployment

See `docs/DEPLOYMENT.md` for the current deploy and verification commands.
