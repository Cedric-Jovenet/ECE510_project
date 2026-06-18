#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>

#if __has_include("config_local.h")
#include "config_local.h"
#endif

#ifndef DEVICE_ID
#define DEVICE_ID "worker-tag-1"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

#ifndef BASE_STATION_URL
#define BASE_STATION_URL "http://10.152.83.65:8090"
#endif

namespace {
constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;

constexpr int GPS_RX = 16;
constexpr int GPS_TX = 17;

constexpr int LORA_SCK = 18;
constexpr int LORA_MISO = 19;
constexpr int LORA_MOSI = 23;
constexpr int LORA_CS = 5;
constexpr int LORA_RST = 14;
constexpr int LORA_DIO0 = 26;

constexpr float LORA_FREQUENCY_MHZ = 915.0f;
constexpr uint32_t PING_PERIOD_MS = 5000;
constexpr uint32_t REPORT_COOLDOWN_MS = 30000;
constexpr uint32_t ALERT_HOLD_MS = 30000;
constexpr uint32_t WIFI_RETRY_MS = 15000;
constexpr float WARNING_G = 1.7f;
constexpr float IMPACT_G = 2.7f;
constexpr float FREEFALL_G = 0.35f;
constexpr uint32_t FREEFALL_WINDOW_MS = 2000;

constexpr uint8_t REG_FIFO = 0x00;
constexpr uint8_t REG_OP_MODE = 0x01;
constexpr uint8_t REG_FRF_MSB = 0x06;
constexpr uint8_t REG_PA_CONFIG = 0x09;
constexpr uint8_t REG_LNA = 0x0C;
constexpr uint8_t REG_FIFO_ADDR_PTR = 0x0D;
constexpr uint8_t REG_FIFO_TX_BASE_ADDR = 0x0E;
constexpr uint8_t REG_FIFO_RX_BASE_ADDR = 0x0F;
constexpr uint8_t REG_IRQ_FLAGS = 0x12;
constexpr uint8_t REG_MODEM_CONFIG_1 = 0x1D;
constexpr uint8_t REG_MODEM_CONFIG_2 = 0x1E;
constexpr uint8_t REG_PREAMBLE_MSB = 0x20;
constexpr uint8_t REG_PREAMBLE_LSB = 0x21;
constexpr uint8_t REG_PAYLOAD_LENGTH = 0x22;
constexpr uint8_t REG_SYNC_WORD = 0x39;
constexpr uint8_t REG_DIO_MAPPING_1 = 0x40;
constexpr uint8_t REG_VERSION = 0x42;

constexpr uint8_t MODE_LONG_RANGE = 0x80;
constexpr uint8_t MODE_SLEEP = 0x00;
constexpr uint8_t MODE_STDBY = 0x01;
constexpr uint8_t MODE_TX = 0x03;
constexpr uint8_t IRQ_TX_DONE = 0x08;

struct ImuState {
  bool ok = false;
  uint8_t addr = 0;
  float g = 0.0f;
  float maxG = 0.0f;
  uint32_t lastFreefallMs = 0;
};

struct GpsState {
  bool seen = false;
  bool fix = false;
  double lat = 0.0;
  double lon = 0.0;
  uint32_t bytes = 0;
  uint32_t lastFixMs = 0;
  char line[128] = {0};
  size_t lineLen = 0;
};

ImuState imu;
GpsState gps;
bool loraOk = false;
uint32_t sequenceNumber = 0;
uint32_t lastPingMs = 0;
uint32_t lastReportMs = 0;
uint32_t alertUntilMs = 0;
uint32_t lastWifiAttemptMs = 0;
String alertReason;
String serialLine;

uint8_t loraReadReg(uint8_t addr) {
  digitalWrite(LORA_CS, LOW);
  SPI.transfer(addr & 0x7F);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(LORA_CS, HIGH);
  return value;
}

void loraWriteReg(uint8_t addr, uint8_t value) {
  digitalWrite(LORA_CS, LOW);
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  digitalWrite(LORA_CS, HIGH);
}

void resetLora() {
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);
  delay(20);
  digitalWrite(LORA_RST, LOW);
  delay(20);
  digitalWrite(LORA_RST, HIGH);
  delay(120);
}

void configureLora() {
  loraWriteReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_SLEEP);
  delay(10);

  uint32_t frf = static_cast<uint32_t>((LORA_FREQUENCY_MHZ * 1000000.0f) / 61.03515625f);
  loraWriteReg(REG_FRF_MSB, static_cast<uint8_t>((frf >> 16) & 0xFF));
  loraWriteReg(REG_FRF_MSB + 1, static_cast<uint8_t>((frf >> 8) & 0xFF));
  loraWriteReg(REG_FRF_MSB + 2, static_cast<uint8_t>(frf & 0xFF));
  loraWriteReg(REG_FIFO_TX_BASE_ADDR, 0x00);
  loraWriteReg(REG_FIFO_RX_BASE_ADDR, 0x00);
  loraWriteReg(REG_LNA, loraReadReg(REG_LNA) | 0x03);
  loraWriteReg(REG_MODEM_CONFIG_1, 0x72);
  loraWriteReg(REG_MODEM_CONFIG_2, 0x74);
  loraWriteReg(REG_PREAMBLE_MSB, 0x00);
  loraWriteReg(REG_PREAMBLE_LSB, 0x08);
  loraWriteReg(REG_SYNC_WORD, 0x34);
  loraWriteReg(REG_DIO_MAPPING_1, 0x40);
  loraWriteReg(REG_PA_CONFIG, 0x8F);
  loraWriteReg(REG_IRQ_FLAGS, 0xFF);
  loraWriteReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY);
}

bool initLora() {
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  pinMode(LORA_DIO0, INPUT);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  resetLora();
  uint8_t version = loraReadReg(REG_VERSION);
  if (version != 0x12) {
    Serial.printf("[LoRa] NOT FOUND RegVersion=0x%02X\n", version);
    return false;
  }
  configureLora();
  Serial.println("[LoRa] OK RFM95/SX127x 915MHz SF7 BW125 sync=0x34");
  return true;
}

bool sendLora(const String& payload) {
  if (!loraOk || payload.length() > 250) {
    return false;
  }
  loraWriteReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY);
  loraWriteReg(REG_FIFO_ADDR_PTR, 0x00);
  loraWriteReg(REG_IRQ_FLAGS, 0xFF);
  for (size_t i = 0; i < payload.length(); ++i) {
    loraWriteReg(REG_FIFO, static_cast<uint8_t>(payload[i]));
  }
  loraWriteReg(REG_PAYLOAD_LENGTH, static_cast<uint8_t>(payload.length()));
  loraWriteReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_TX);

  uint32_t deadline = millis() + 2000;
  while (millis() < deadline) {
    uint8_t flags = loraReadReg(REG_IRQ_FLAGS);
    if (flags & IRQ_TX_DONE) {
      loraWriteReg(REG_IRQ_FLAGS, IRQ_TX_DONE);
      loraWriteReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY);
      return true;
    }
    delay(2);
  }
  loraWriteReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY);
  return false;
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(static_cast<int>(addr), static_cast<int>(len)) != static_cast<int>(len)) {
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    data[i] = Wire.read();
  }
  return true;
}

bool initImu() {
  const uint8_t addrs[] = {0x68, 0x69};
  for (uint8_t addr : addrs) {
    if (!i2cPresent(addr)) {
      continue;
    }
    imu.addr = addr;
    i2cWrite8(addr, 0x6B, 0x00);
    delay(50);
    i2cWrite8(addr, 0x1C, 0x10);
    imu.ok = true;
    Serial.printf("[IMU] OK MPU/ICM addr=0x%02X accel=+/-8g\n", addr);
    return true;
  }
  Serial.println("[IMU] not found at 0x68/0x69");
  return false;
}

bool updateImu() {
  if (!imu.ok) {
    return false;
  }
  uint8_t raw[6];
  if (!i2cRead(imu.addr, 0x3B, raw, sizeof(raw))) {
    imu.ok = false;
    return false;
  }
  int16_t ax = static_cast<int16_t>((raw[0] << 8) | raw[1]);
  int16_t ay = static_cast<int16_t>((raw[2] << 8) | raw[3]);
  int16_t az = static_cast<int16_t>((raw[4] << 8) | raw[5]);
  float gx = static_cast<float>(ax) / 4096.0f;
  float gy = static_cast<float>(ay) / 4096.0f;
  float gz = static_cast<float>(az) / 4096.0f;
  imu.g = sqrtf(gx * gx + gy * gy + gz * gz);
  if (imu.g > imu.maxG) {
    imu.maxG = imu.g;
  }
  if (imu.g <= FREEFALL_G) {
    imu.lastFreefallMs = millis();
  }
  return true;
}

double nmeaCoordToDeg(const char* value, const char* hemisphere) {
  if (value == nullptr || value[0] == '\0') {
    return 0.0;
  }
  double raw = atof(value);
  int degrees = static_cast<int>(raw / 100.0);
  double minutes = raw - (degrees * 100.0);
  double result = degrees + minutes / 60.0;
  if (hemisphere != nullptr && (hemisphere[0] == 'S' || hemisphere[0] == 'W')) {
    result = -result;
  }
  return result;
}

bool getNmeaField(char* line, int target, char* out, size_t outLen) {
  int field = 0;
  size_t pos = 0;
  for (size_t i = 0; line[i] != '\0'; ++i) {
    char c = line[i];
    if (c == ',' || c == '*') {
      if (field == target) {
        out[pos] = '\0';
        return true;
      }
      ++field;
      pos = 0;
      continue;
    }
    if (field == target && pos + 1 < outLen) {
      out[pos++] = c;
    }
  }
  if (field == target) {
    out[pos] = '\0';
    return true;
  }
  return false;
}

void parseGpsLine(char* line) {
  if (strncmp(line, "$GPGGA", 6) == 0 || strncmp(line, "$GNGGA", 6) == 0) {
    char lat[18], ns[3], lon[18], ew[3], fix[4];
    if (getNmeaField(line, 2, lat, sizeof(lat)) &&
        getNmeaField(line, 3, ns, sizeof(ns)) &&
        getNmeaField(line, 4, lon, sizeof(lon)) &&
        getNmeaField(line, 5, ew, sizeof(ew)) &&
        getNmeaField(line, 6, fix, sizeof(fix))) {
      gps.seen = true;
      gps.fix = atoi(fix) > 0;
      if (gps.fix) {
        gps.lat = nmeaCoordToDeg(lat, ns);
        gps.lon = nmeaCoordToDeg(lon, ew);
        gps.lastFixMs = millis();
      }
    }
  } else if (strncmp(line, "$GPRMC", 6) == 0 || strncmp(line, "$GNRMC", 6) == 0) {
    char status[3], lat[18], ns[3], lon[18], ew[3];
    if (getNmeaField(line, 2, status, sizeof(status)) &&
        getNmeaField(line, 3, lat, sizeof(lat)) &&
        getNmeaField(line, 4, ns, sizeof(ns)) &&
        getNmeaField(line, 5, lon, sizeof(lon)) &&
        getNmeaField(line, 6, ew, sizeof(ew))) {
      gps.seen = true;
      gps.fix = status[0] == 'A';
      if (gps.fix) {
        gps.lat = nmeaCoordToDeg(lat, ns);
        gps.lon = nmeaCoordToDeg(lon, ew);
        gps.lastFixMs = millis();
      }
    }
  }
}

void updateGps() {
  while (Serial2.available()) {
    char c = static_cast<char>(Serial2.read());
    ++gps.bytes;
    if (c == '\n') {
      gps.line[gps.lineLen] = '\0';
      if (gps.lineLen > 6) {
        parseGpsLine(gps.line);
      }
      gps.lineLen = 0;
    } else if (c != '\r' && gps.lineLen + 1 < sizeof(gps.line)) {
      gps.line[gps.lineLen++] = c;
    }
  }
}

String levelText() {
  if (millis() < alertUntilMs) {
    return "critical";
  }
  if (imu.ok && imu.g >= WARNING_G) {
    return "warning";
  }
  return "clear";
}

String baseJson(const char* type, const char* level) {
  String payload;
  payload.reserve(230);
  payload += "{\"type\":\"";
  payload += type;
  payload += "\",\"device_id\":\"";
  payload += DEVICE_ID;
  payload += "\",\"device_type\":\"worker_tag\",\"level\":\"";
  payload += level;
  payload += "\",\"seq\":";
  payload += String(++sequenceNumber);
  payload += ",\"uptime_ms\":";
  payload += String(millis());
  payload += ",\"imu_ok\":";
  payload += imu.ok ? "true" : "false";
  payload += ",\"g\":";
  payload += String(imu.g, 2);
  payload += ",\"gps_seen\":";
  payload += gps.seen ? "true" : "false";
  payload += ",\"gps_fix\":";
  payload += gps.fix ? "true" : "false";
  if (gps.fix) {
    payload += ",\"lat\":";
    payload += String(gps.lat, 6);
    payload += ",\"lon\":";
    payload += String(gps.lon, 6);
  }
  return payload;
}

bool postHttp(const char* path, const String& payload) {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }
  HTTPClient http;
  String url = String(BASE_STATION_URL) + path;
  if (!http.begin(url)) {
    return false;
  }
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(payload);
  http.end();
  return code >= 200 && code < 300;
}

bool sendPacket(const char* path, const String& payload) {
  bool sentLora = sendLora(payload);
  bool sentHttp = false;
  if (!sentLora) {
    sentHttp = postHttp(path, payload);
  }
  Serial.printf("[TX] %s len=%u lora=%s http=%s\n",
                path,
                static_cast<unsigned>(payload.length()),
                sentLora ? "ok" : "fail",
                sentHttp ? "ok" : "skip/fail");
  return sentLora || sentHttp;
}

void sendPing() {
  String level = levelText();
  String payload = baseJson("worker_ping", level.c_str());
  payload += "}";
  sendPacket("/api/ping", payload);
}

void sendReport(const char* reason) {
  String payload = baseJson("accident_report", "critical");
  payload += ",\"reason\":\"";
  payload += reason;
  payload += "\",\"max_g\":";
  payload += String(imu.maxG, 2);
  payload += "}";
  if (sendPacket("/api/report", payload)) {
    lastReportMs = millis();
  }
}

void triggerAlert(const char* reason) {
  alertReason = reason;
  alertUntilMs = millis() + ALERT_HOLD_MS;
  sendReport(reason);
}

void maybeDetectAccident() {
  if (!imu.ok) {
    return;
  }
  uint32_t now = millis();
  if (imu.g >= IMPACT_G) {
    triggerAlert("impact");
    return;
  }
  bool recentFreefall = imu.lastFreefallMs != 0 && now - imu.lastFreefallMs <= FREEFALL_WINDOW_MS;
  if (recentFreefall && imu.g >= WARNING_G) {
    triggerAlert("freefall_impact");
  }
}

void maybeRepeatReport() {
  uint32_t now = millis();
  if (now >= alertUntilMs) {
    return;
  }
  if (lastReportMs == 0 || now - lastReportMs >= REPORT_COOLDOWN_MS) {
    sendReport(alertReason.length() ? alertReason.c_str() : "still_critical");
  }
}

void maintainWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  uint32_t now = millis();
  if (now - lastWifiAttemptMs < WIFI_RETRY_MS) {
    return;
  }
  lastWifiAttemptMs = now;
  WiFi.disconnect(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("[WiFi] reconnect requested");
}

void printStatus() {
  Serial.printf("[status] id=%s lora=%s wifi=%s imu=%s g=%.2f gps_seen=%s gps_fix=%s seq=%lu level=%s\n",
                DEVICE_ID,
                loraOk ? "ok" : "fail",
                WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "down",
                imu.ok ? "ok" : "fail",
                imu.g,
                gps.seen ? "yes" : "no",
                gps.fix ? "yes" : "no",
                static_cast<unsigned long>(sequenceNumber),
                levelText().c_str());
}

void handleSerialLine(String line) {
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) {
    return;
  }
  if (line == "report" || line == "accident") {
    triggerAlert("serial");
  } else if (line == "clear") {
    alertUntilMs = 0;
    alertReason = "";
    Serial.println("[alert] cleared");
  } else if (line == "ping") {
    sendPing();
  } else if (line == "status") {
    printStatus();
  } else if (line == "help") {
    Serial.println("commands: status, ping, report, clear, help");
  } else {
    Serial.println("unknown command; type help");
  }
}

void updateSerialCommands() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      handleSerialLine(serialLine);
      serialLine = "";
    } else if (serialLine.length() < 80) {
      serialLine += c;
    }
  }
}
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== ECE510 worker tag firmware ===");
  Serial.printf("device_id=%s base=%s\n", DEVICE_ID, BASE_STATION_URL);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  initImu();

  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.printf("[GPS] UART2 RX=%d TX=%d baud=9600\n", GPS_RX, GPS_TX);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWifiAttemptMs = millis();

  loraOk = initLora();
  sendPing();
}

void loop() {
  updateSerialCommands();
  updateGps();
  updateImu();
  maintainWifi();
  maybeDetectAccident();
  maybeRepeatReport();

  uint32_t now = millis();
  if (now - lastPingMs >= PING_PERIOD_MS) {
    lastPingMs = now;
    sendPing();
    printStatus();
  }
  delay(50);
}
