#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

void initTelemetry();
void sendTelemetry();
void sendTelemetryTask();

// Thread-safe BLE notify — use this instead of calling
// pTelemetryCharacteristic->notify() directly anywhere in the codebase.
void bleSendResponse(const uint8_t* data, size_t length);

#endif // TELEMETRY_H
