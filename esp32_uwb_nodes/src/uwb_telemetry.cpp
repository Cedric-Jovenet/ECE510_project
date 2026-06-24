#include "uwb_telemetry.h"
#include "uwb_lora.h"

#ifndef UWB_WIFI_TELEMETRY
#define UWB_WIFI_TELEMETRY 0
#endif

#ifndef UWB_IMU_TELEMETRY
#define UWB_IMU_TELEMETRY 0
#endif

#if UWB_WIFI_TELEMETRY

#include <WiFi.h>
#include <WiFiUdp.h>

#if UWB_IMU_TELEMETRY
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#endif

#include "config.h"

#if __has_include("wifi_config.h")
#include "wifi_config.h"
#else
#include "wifi_config.example.h"
#endif

namespace {

const uint32_t USB_HOST_TIMEOUT_MS = 6000;
const uint32_t WIFI_START_GRACE_MS = 5000;
const uint32_t WIFI_RECONNECT_MS = 5000;
const uint16_t LOCAL_UDP_PORT = 8891 + NODE_ID;

WiFiUDP udp;
IPAddress remoteIp;
bool udpStarted = false;
bool wifiStarted = false;
bool wifiWasConnected = false;
uint32_t bootMs = 0;
uint32_t lastUsbHeartbeatMs = 0;
uint32_t lastReconnectMs = 0;
uint32_t lastImuMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t sequenceNumber = 0;
char serialCommand[32] = {};
size_t serialCommandLength = 0;

#if UWB_IMU_TELEMETRY
Adafruit_MPU6050 imu;
bool imuAvailable = false;
#else
const bool imuAvailable = false;
#endif

const char *nodeRole()
{
    return NODE_ID == INITIATOR_NODE_ID ? "initiator" : "responder";
}

bool wifiConnected()
{
    return WiFi.status() == WL_CONNECTED;
}

bool usbHostActive()
{
    return lastUsbHeartbeatMs != 0 &&
           millis() - lastUsbHeartbeatMs < USB_HOST_TIMEOUT_MS;
}

void stopWifiForUsb()
{
    if (wifiStarted) {
        udp.stop();
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        udpStarted = false;
        wifiStarted = false;
        wifiWasConnected = false;
        Serial.printf("[NODE] transport=usb node_id=%u\n",
                      static_cast<unsigned>(NODE_ID));
    }
}

void startWifi()
{
    if (wifiStarted || strlen(UWB_WIFI_SSID) == 0) {
        return;
    }

    Serial.printf("[NODE] Connecting WiFi node_id=%u SSID=%s UDP=%s:%u\n",
                  static_cast<unsigned>(NODE_ID),
                  UWB_WIFI_SSID,
                  UWB_UDP_HOST,
                  static_cast<unsigned>(UWB_UDP_PORT));
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(UWB_WIFI_SSID, UWB_WIFI_PASSWORD);
    wifiStarted = true;
    lastReconnectMs = millis();
}

void processSerialInput()
{
    while (Serial.available() > 0) {
        const char value = static_cast<char>(Serial.read());
        if (value == '\r' || value == '\n') {
            if (serialCommandLength > 0) {
                serialCommand[serialCommandLength] = '\0';
                if (strcmp(serialCommand, "HOST_USB") == 0) {
                    if (lastUsbHeartbeatMs == 0) {
                        Serial.printf("[NODE] transport=usb node_id=%u\n",
                                      static_cast<unsigned>(NODE_ID));
                    }
                    lastUsbHeartbeatMs = millis();
                }
                serialCommandLength = 0;
            }
            continue;
        }
        if (serialCommandLength < sizeof(serialCommand) - 1) {
            serialCommand[serialCommandLength++] = value;
        } else {
            serialCommandLength = 0;
        }
    }
}

void updateTransport()
{
    processSerialInput();
    if (usbHostActive()) {
        stopWifiForUsb();
        return;
    }
    if (millis() - bootMs < WIFI_START_GRACE_MS) {
        return;
    }

    startWifi();
    const bool connected = wifiConnected();
    if (connected && !wifiWasConnected) {
        udpStarted = false;
        Serial.printf("[NODE] transport=wifi node_id=%u ip=%s rssi=%d\n",
                      static_cast<unsigned>(NODE_ID),
                      WiFi.localIP().toString().c_str(),
                      WiFi.RSSI());
    } else if (!connected && wifiWasConnected) {
        udp.stop();
        udpStarted = false;
        Serial.printf("[NODE] WiFi disconnected node_id=%u; UWB remains active\n",
                      static_cast<unsigned>(NODE_ID));
    }
    wifiWasConnected = connected;

    if (!connected && wifiStarted &&
        millis() - lastReconnectMs >= WIFI_RECONNECT_MS) {
        Serial.printf("[NODE] WiFi retry node_id=%u status=%d\n",
                      static_cast<unsigned>(NODE_ID),
                      static_cast<int>(WiFi.status()));
        WiFi.disconnect(false);
        WiFi.begin(UWB_WIFI_SSID, UWB_WIFI_PASSWORD);
        lastReconnectMs = millis();
    }
}

bool sendPacket(const char *payload)
{
    if (!wifiConnected() || usbHostActive()) {
        return false;
    }
    if (!udpStarted) {
        udpStarted = udp.begin(LOCAL_UDP_PORT) == 1;
        if (!udpStarted) {
            return false;
        }
    }
    if (!udp.beginPacket(remoteIp, UWB_UDP_PORT)) {
        return false;
    }
    udp.write(reinterpret_cast<const uint8_t *>(payload), strlen(payload));
    return udp.endPacket() == 1;
}

void processRelayPackets()
{
    if (!udpStarted || !wifiConnected() || usbHostActive()) {
        return;
    }
    const int packetSize = udp.parsePacket();
    if (packetSize <= 0) {
        return;
    }

    char payload[251] = {};
    const int length = udp.read(
        reinterpret_cast<uint8_t *>(payload),
        sizeof(payload) - 1);
    if (length <= 0) {
        return;
    }
    payload[length] = '\0';
    if (strstr(payload, "\"type\":\"machine_ping\"") != nullptr &&
        strstr(payload, "\"device_id\":\"machine-1\"") != nullptr) {
        uwbLoraSendRelay(payload);
    }
}

void publishHeartbeat()
{
    if (millis() - lastHeartbeatMs < 2000) {
        return;
    }
    lastHeartbeatMs = millis();

    char payload[320];
    snprintf(payload, sizeof(payload),
             "{\"type\":\"node_status\",\"node_id\":%u,"
             "\"role\":\"%s\",\"transport\":\"wifi\","
             "\"worker_id\":\"%s\",\"wifi\":%s,\"imu\":%s,"
             "\"rssi\":%d,\"seq\":%lu,\"uptime_ms\":%lu}",
             static_cast<unsigned>(NODE_ID),
             nodeRole(),
             UWB_WORKER_ID,
             wifiConnected() ? "true" : "false",
             imuAvailable ? "true" : "false",
             wifiConnected() ? WiFi.RSSI() : 0,
             static_cast<unsigned long>(sequenceNumber++),
             static_cast<unsigned long>(millis()));
    sendPacket(payload);
}

#if UWB_IMU_TELEMETRY
void publishImu()
{
    if (!imuAvailable || millis() - lastImuMs < 250) {
        return;
    }
    lastImuMs = millis();

    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;
    imu.getEvent(&acceleration, &gyro, &temperature);

    char payload[384];
    snprintf(payload, sizeof(payload),
             "{\"type\":\"imu\",\"worker_id\":\"%s\","
             "\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,"
             "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f,"
             "\"seq\":%lu,\"uptime_ms\":%lu}",
             UWB_WORKER_ID,
             acceleration.acceleration.x,
             acceleration.acceleration.y,
             acceleration.acceleration.z,
             gyro.gyro.x,
             gyro.gyro.y,
             gyro.gyro.z,
             static_cast<unsigned long>(sequenceNumber++),
             static_cast<unsigned long>(millis()));
    sendPacket(payload);
}
#else
void publishImu() {}
#endif

}  // namespace

void uwbTelemetryBegin()
{
    bootMs = millis();
    if (!remoteIp.fromString(UWB_UDP_HOST)) {
        Serial.printf("[NODE] Invalid UDP host: %s\n", UWB_UDP_HOST);
    }

#if UWB_IMU_TELEMETRY
    Wire.begin(21, 22);
    imuAvailable = imu.begin();
    if (imuAvailable) {
        imu.setAccelerometerRange(MPU6050_RANGE_8_G);
        imu.setGyroRange(MPU6050_RANGE_500_DEG);
        imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.println("[NODE] MPU6050 ready");
    } else {
        Serial.println("[NODE] MPU6050 not found; UWB remains active");
    }
#endif

    Serial.printf("[NODE] Waiting %lu ms for USB host heartbeat node_id=%u\n",
                  static_cast<unsigned long>(WIFI_START_GRACE_MS),
                  static_cast<unsigned>(NODE_ID));
}

void uwbTelemetryLoop()
{
    updateTransport();
    processRelayPackets();
    publishImu();
    publishHeartbeat();
}

void uwbTelemetryPublishDistance(uint8_t anchorId, double distanceM)
{
    char payload[320];
    snprintf(payload, sizeof(payload),
             "{\"type\":\"uwb_distance\",\"worker_id\":\"%s\","
             "\"node_id\":%u,\"anchor_id\":%u,\"distance_m\":%.4f,"
             "\"seq\":%lu,\"uptime_ms\":%lu}",
             UWB_WORKER_ID,
             static_cast<unsigned>(NODE_ID),
             static_cast<unsigned>(anchorId),
             distanceM,
             static_cast<unsigned long>(sequenceNumber++),
             static_cast<unsigned long>(millis()));
    sendPacket(payload);
}

#else

void uwbTelemetryBegin() {}
void uwbTelemetryLoop() {}
void uwbTelemetryPublishDistance(uint8_t, double) {}

#endif
