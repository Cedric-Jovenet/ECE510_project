#include "uwb_lora.h"

#ifndef UWB_LORA_TELEMETRY
#define UWB_LORA_TELEMETRY 0
#endif

#if UWB_LORA_TELEMETRY

#include <SPI.h>

#include "config.h"

#if __has_include("wifi_config.h")
#include "wifi_config.h"
#else
#include "wifi_config.example.h"
#endif

namespace {

const int UWB_CS_PIN = 4;
const int LORA_CS_PIN = 5;
const int LORA_RESET_PIN = 14;
const uint32_t LORA_RETRY_MS = 10000;
const uint32_t LORA_PING_MS = 5000;
const float ALERT_DISTANCE_M = 2.0f;

const uint8_t REG_FIFO = 0x00;
const uint8_t REG_OP_MODE = 0x01;
const uint8_t REG_FRF_MSB = 0x06;
const uint8_t REG_PA_CONFIG = 0x09;
const uint8_t REG_LNA = 0x0C;
const uint8_t REG_FIFO_ADDR_PTR = 0x0D;
const uint8_t REG_FIFO_TX_BASE_ADDR = 0x0E;
const uint8_t REG_FIFO_RX_BASE_ADDR = 0x0F;
const uint8_t REG_IRQ_FLAGS = 0x12;
const uint8_t REG_MODEM_CONFIG_1 = 0x1D;
const uint8_t REG_MODEM_CONFIG_2 = 0x1E;
const uint8_t REG_PREAMBLE_MSB = 0x20;
const uint8_t REG_PREAMBLE_LSB = 0x21;
const uint8_t REG_PAYLOAD_LENGTH = 0x22;
const uint8_t REG_SYNC_WORD = 0x39;
const uint8_t REG_DIO_MAPPING_1 = 0x40;
const uint8_t REG_VERSION = 0x42;
const uint8_t MODE_LONG_RANGE = 0x80;
const uint8_t MODE_SLEEP = 0x00;
const uint8_t MODE_STANDBY = 0x01;
const uint8_t MODE_TX = 0x03;
const uint8_t IRQ_TX_DONE = 0x08;

bool radioReady = false;
uint32_t lastInitAttemptMs = 0;
uint32_t lastPingMs = 0;
uint32_t sequenceNumber = 0;
float rangesM[NUM_NODES + 1] = {};
bool rangeValid[NUM_NODES + 1] = {};

uint8_t readRegister(uint8_t address)
{
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(UWB_CS_PIN, HIGH);
    digitalWrite(LORA_CS_PIN, LOW);
    SPI.transfer(address & 0x7F);
    const uint8_t value = SPI.transfer(0x00);
    digitalWrite(LORA_CS_PIN, HIGH);
    SPI.endTransaction();
    return value;
}

void writeRegister(uint8_t address, uint8_t value)
{
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(UWB_CS_PIN, HIGH);
    digitalWrite(LORA_CS_PIN, LOW);
    SPI.transfer(address | 0x80);
    SPI.transfer(value);
    digitalWrite(LORA_CS_PIN, HIGH);
    SPI.endTransaction();
}

void resetRadio()
{
    pinMode(LORA_RESET_PIN, OUTPUT);
    digitalWrite(LORA_RESET_PIN, HIGH);
    delay(20);
    digitalWrite(LORA_RESET_PIN, LOW);
    delay(100);
    digitalWrite(LORA_RESET_PIN, HIGH);
    delay(150);
}

void configureRadio()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE | MODE_SLEEP);
    delay(10);
    const uint32_t frf = static_cast<uint32_t>(
        (915.0f * 1000000.0f) / 61.03515625f);
    writeRegister(REG_FRF_MSB, (frf >> 16) & 0xFF);
    writeRegister(REG_FRF_MSB + 1, (frf >> 8) & 0xFF);
    writeRegister(REG_FRF_MSB + 2, frf & 0xFF);
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0x00);
    writeRegister(REG_FIFO_ADDR_PTR, 0x00);
    writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);
    writeRegister(REG_MODEM_CONFIG_1, 0x72);
    writeRegister(REG_MODEM_CONFIG_2, 0x74);
    writeRegister(REG_PREAMBLE_MSB, 0x00);
    writeRegister(REG_PREAMBLE_LSB, 0x08);
    writeRegister(REG_SYNC_WORD, 0x34);
    writeRegister(REG_DIO_MAPPING_1, 0x40);
    writeRegister(REG_PA_CONFIG, 0x8F);
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE | MODE_STANDBY);
}

bool initializeRadio()
{
    lastInitAttemptMs = millis();
    pinMode(UWB_CS_PIN, OUTPUT);
    digitalWrite(UWB_CS_PIN, HIGH);
    pinMode(LORA_CS_PIN, OUTPUT);
    digitalWrite(LORA_CS_PIN, HIGH);
    resetRadio();
    const uint8_t version = readRegister(REG_VERSION);
    if (version != 0x12) {
        Serial.printf(
            "[LoRa] unavailable RegVersion=0x%02X; WiFi fallback active\n",
            version);
        return false;
    }
    configureRadio();
    Serial.println(
        "[LoRa] ready RFM95 915MHz; direct worker pings enabled");
    return true;
}

bool sendPacket(const char *payload)
{
    const size_t length = strlen(payload);
    if (!radioReady || length == 0 || length > 250) {
        return false;
    }

    writeRegister(REG_OP_MODE, MODE_LONG_RANGE | MODE_STANDBY);
    writeRegister(REG_FIFO_ADDR_PTR, 0x00);
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    for (size_t index = 0; index < length; ++index) {
        writeRegister(REG_FIFO, static_cast<uint8_t>(payload[index]));
    }
    writeRegister(REG_PAYLOAD_LENGTH, static_cast<uint8_t>(length));
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE | MODE_TX);

    const uint32_t deadline = millis() + 1200;
    while (static_cast<int32_t>(deadline - millis()) > 0) {
        const uint8_t flags = readRegister(REG_IRQ_FLAGS);
        if (flags & IRQ_TX_DONE) {
            writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE);
            writeRegister(REG_OP_MODE, MODE_LONG_RANGE | MODE_STANDBY);
            return true;
        }
        delay(1);
    }
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE | MODE_STANDBY);
    return false;
}

void publishWorkerPing()
{
    if (!radioReady || millis() - lastPingMs < LORA_PING_MS) {
        return;
    }
    lastPingMs = millis();

    float minimumRange = 10000.0f;
    for (uint8_t anchor = 1; anchor < INITIATOR_NODE_ID; ++anchor) {
        if (rangeValid[anchor] && rangesM[anchor] < minimumRange) {
            minimumRange = rangesM[anchor];
        }
    }
    const char *level = (
        minimumRange <= ALERT_DISTANCE_M ? "critical" : "clear");

    char payload[240];
    snprintf(
        payload,
        sizeof(payload),
        "{\"type\":\"worker_ping\",\"device_id\":\"%s\","
        "\"device_type\":\"worker\",\"level\":\"%s\","
        "\"node_id\":%u,\"seq\":%lu,\"r1\":%.3f,\"r2\":%.3f,"
        "\"r3\":%.3f}",
        UWB_WORKER_ID,
        level,
        static_cast<unsigned>(NODE_ID),
        static_cast<unsigned long>(++sequenceNumber),
        rangeValid[1] ? rangesM[1] : -1.0f,
        rangeValid[2] ? rangesM[2] : -1.0f,
        rangeValid[3] ? rangesM[3] : -1.0f);
    const bool sent = sendPacket(payload);
    Serial.printf(
        "[LoRa] worker ping %s seq=%lu\n",
        sent ? "sent" : "failed",
        static_cast<unsigned long>(sequenceNumber));
}

}  // namespace

void uwbLoraBegin()
{
    radioReady = initializeRadio();
}

void uwbLoraLoop()
{
    if (!radioReady) {
        if (millis() - lastInitAttemptMs >= LORA_RETRY_MS) {
            radioReady = initializeRadio();
        }
        return;
    }
    publishWorkerPing();
}

void uwbLoraRecordDistance(uint8_t anchorId, double distanceM)
{
    if (anchorId == 0 || anchorId >= INITIATOR_NODE_ID) {
        return;
    }
    rangesM[anchorId] = static_cast<float>(distanceM);
    rangeValid[anchorId] = true;
}

bool uwbLoraSendRelay(const char *payload)
{
    const bool sent = sendPacket(payload);
    Serial.printf("[LoRa] machine relay %s\n", sent ? "sent" : "failed");
    return sent;
}

#else

void uwbLoraBegin() {}
void uwbLoraLoop() {}
void uwbLoraRecordDistance(uint8_t, double) {}
bool uwbLoraSendRelay(const char *) { return false; }

#endif
