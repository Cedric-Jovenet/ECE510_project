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
