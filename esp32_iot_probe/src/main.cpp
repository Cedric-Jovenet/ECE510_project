// Hardware probe sketch for the ESP32 IoT carrier.
// It checks expected I2C, LoRa, and GPS wiring before flashing the real
// worker-tag firmware.
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

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

uint8_t i2cRead8(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return 0xff;
  }
  if (Wire.requestFrom(static_cast<int>(addr), 1) != 1) {
    return 0xff;
  }
  return Wire.read();
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void scanExpectedI2c() {
  Serial.println();
  Serial.println("[I2C] expected IMU addresses SDA=21 SCL=22");
  Serial.printf("  line state before transactions: SDA=%d SCL=%d\n", digitalRead(I2C_SDA), digitalRead(I2C_SCL));
  int found = 0;
  const uint8_t expected[] = {0x68, 0x69, 0x53};
  for (uint8_t addr : expected) {
    if (i2cPresent(addr)) {
      Serial.printf("  found 0x%02X\n", addr);
      ++found;
    } else {
      Serial.printf("  no ack 0x%02X\n", addr);
    }
  }
  if (!found) {
    Serial.println("  no expected MPU6050/ICM/ADXL345 address found");
  }
}

void probeKnownI2cChips() {
  Serial.println();
  Serial.println("[I2C] known chip registers");
  bool known = false;
  const uint8_t mpuAddrs[] = {0x68, 0x69};
  for (uint8_t addr : mpuAddrs) {
    if (i2cPresent(addr)) {
      known = true;
      Serial.printf("  MPU/ICM candidate 0x%02X WHO_AM_I(0x75)=0x%02X PWR_MGMT_1(0x6B)=0x%02X\n",
                    addr, i2cRead8(addr, 0x75), i2cRead8(addr, 0x6B));
    }
  }
  if (i2cPresent(0x53)) {
    known = true;
    Serial.printf("  ADXL345 candidate 0x53 DEVID(0x00)=0x%02X\n", i2cRead8(0x53, 0x00));
  }
  if (!known) {
    Serial.println("  no MPU6050/ICM at 0x68/0x69 and no ADXL345 at 0x53");
  }
}

uint8_t loraReadReg(uint8_t addr) {
  digitalWrite(LORA_CS, LOW);
  SPI.transfer(addr & 0x7f);
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
  delay(100);
}

void probeLora() {
  Serial.println();
  Serial.println("[LoRa] probe RFM95/RFM96 SX127x");
  Serial.printf("  pins SCK=%d MISO=%d MOSI=%d CS=%d RST=%d DIO0=%d\n",
                LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS, LORA_RST, LORA_DIO0);
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  pinMode(LORA_DIO0, INPUT);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  resetLora();

  uint8_t version = loraReadReg(0x42);
  uint8_t opMode = loraReadReg(0x01);
  uint8_t irqFlags = loraReadReg(0x12);
  Serial.printf("  RegVersion(0x42)=0x%02X expected 0x12\n", version);
  Serial.printf("  RegOpMode(0x01)=0x%02X RegIrqFlags(0x12)=0x%02X DIO0=%d\n",
                opMode, irqFlags, digitalRead(LORA_DIO0));

  if (version == 0x12) {
    uint8_t oldSync = loraReadReg(0x39);
    loraWriteReg(0x39, 0x34);
    uint8_t newSync = loraReadReg(0x39);
    loraWriteReg(0x39, oldSync);
    Serial.printf("  SPI read/write OK: SyncWord old=0x%02X test=0x%02X restored\n", oldSync, newSync);
  } else {
    Serial.println("  NOT DETECTED: check 3.3V, GND, SPI pins, CS/RST, and module reset state");
  }
  SPI.endTransaction();
}

uint32_t probeGpsAtBaud(uint32_t baud, uint32_t windowMs) {
  Serial.printf("\n[GPS] UART2 RX=%d TX=%d baud=%lu window=%lums\n", GPS_RX, GPS_TX,
                static_cast<unsigned long>(baud), static_cast<unsigned long>(windowMs));
  Serial2.end();
  delay(50);
  Serial2.begin(baud, SERIAL_8N1, GPS_RX, GPS_TX);
  uint32_t start = millis();
  uint32_t bytes = 0;
  uint32_t lines = 0;
  String firstLine;
  String currentLine;
  while (millis() - start < windowMs) {
    while (Serial2.available()) {
      char c = static_cast<char>(Serial2.read());
      ++bytes;
      if (c == '\n') {
        ++lines;
        currentLine.trim();
        if (firstLine.length() == 0 && currentLine.length() > 0) {
          firstLine = currentLine;
        }
        currentLine = "";
      } else if (c != '\r' && currentLine.length() < 160) {
        currentLine += c;
      }
    }
    delay(10);
  }
  Serial.printf("  bytes=%lu lines=%lu\n", static_cast<unsigned long>(bytes),
                static_cast<unsigned long>(lines));
  if (firstLine.length()) {
    Serial.print("  first NMEA: ");
    Serial.println(firstLine);
  } else if (currentLine.length()) {
    Serial.print("  partial: ");
    Serial.println(currentLine);
  } else {
    Serial.println("  no serial data");
  }
  return bytes;
}

void runProbe() {
  Serial.println();
  Serial.println("=== ESP32 IOT PROBE START ===");
  Serial.printf("build %s %s\n", __DATE__, __TIME__);
  probeLora();
  uint32_t gpsBytes = probeGpsAtBaud(9600, 12000);
  if (gpsBytes == 0) {
    gpsBytes = probeGpsAtBaud(38400, 5000);
  }
  if (gpsBytes == 0) {
    gpsBytes = probeGpsAtBaud(4800, 5000);
  }
  if (gpsBytes == 0) {
    gpsBytes = probeGpsAtBaud(115200, 5000);
  }
  scanExpectedI2c();
  probeKnownI2cChips();
  Serial.println();
  Serial.println("=== ESP32 IOT PROBE END ===");
}
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delay(20);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  runProbe();
}

void loop() {
  delay(30000);
  runProbe();
}
