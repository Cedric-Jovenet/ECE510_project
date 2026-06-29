// Entry point for each ESP32 UWB node. Compile-time build flags choose the
// NODE_ID, initiator/responder role, and optional telemetry transports.
#include <Arduino.h>
#include "uwb_lora.h"
#include "uwb_telemetry.h"
#include "uwb_tdma.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);

    uwbTelemetryBegin();
    uwb_init();
    uwbLoraBegin();
}

void loop()
{
    uwb_loop();
    uwbTelemetryLoop();
    uwbLoraLoop();
}
