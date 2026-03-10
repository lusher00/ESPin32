#include "telemetry.h"
#include "config.h"
#include "ble_handler.h"
#include "motor_control.h"
#include "commands.h"
#include "debug.h"

unsigned long lastTelemetryUpdate = 0;

// Mutex to prevent concurrent BLE notify calls from the telemetry task
// and the command-response path.  portENTER_CRITICAL cannot be used here
// because both callers are FreeRTOS tasks (not ISRs) — using it would
// disable all interrupts (including the encoder ISR) for the duration of
// notify().  A binary semaphore gives proper task-level mutual exclusion.
static SemaphoreHandle_t bleMutex = nullptr;

// Shared response buffer for telemetry — kept static to avoid stack usage.
static uint8_t telemetryPacketBuf[PACKET_MAX_LENGTH];

void initTelemetry() {
  bleMutex = xSemaphoreCreateMutex();
  Serial.println("✓ Telemetry initialized");
}

void sendTelemetry() {
  if (!pTelemetryCharacteristic) return;

  // Build a proper CMD_STATUS_RESPONSE packet so the app can parse it
  // the same way as any other response — no special-casing needed.
  StatusData status;
  getStatusData(status);

  size_t len = buildPacket(telemetryPacketBuf, CMD_STATUS_RESPONSE,
                           (const uint8_t*)&status, sizeof(StatusData));
  if (len == 0) return;

  // Guard against the command-response path also calling notify()
  if (bleMutex && xSemaphoreTake(bleMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pTelemetryCharacteristic->setValue(telemetryPacketBuf, len);
    pTelemetryCharacteristic->notify();
    xSemaphoreGive(bleMutex);
  }

  DEBUG_MOTOR("Telemetry packet sent");
}

void sendTelemetryTask() {
  if (deviceConnected && (millis() - lastTelemetryUpdate >= TELEMETRY_INTERVAL)) {
    sendTelemetry();
    lastTelemetryUpdate = millis();
  }
}

void bleSendResponse(const uint8_t* data, size_t length) {
  if (!pTelemetryCharacteristic || length == 0) return;
  // BLECharacteristic::setValue takes uint8_t* (non-const) despite not
  // modifying the buffer — cast away const to satisfy the library signature.
  if (bleMutex && xSemaphoreTake(bleMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pTelemetryCharacteristic->setValue(const_cast<uint8_t*>(data), length);
    pTelemetryCharacteristic->notify();
    xSemaphoreGive(bleMutex);
  }
}
