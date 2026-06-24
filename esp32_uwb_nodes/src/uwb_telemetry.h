#pragma once

#include <Arduino.h>

void uwbTelemetryBegin();
void uwbTelemetryLoop();
void uwbTelemetryPublishDistance(uint8_t anchorId, double distanceM);
