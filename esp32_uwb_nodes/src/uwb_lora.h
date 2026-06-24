#pragma once

#include <Arduino.h>

void uwbLoraBegin();
void uwbLoraLoop();
void uwbLoraRecordDistance(uint8_t anchorId, double distanceM);
bool uwbLoraSendRelay(const char *payload);
