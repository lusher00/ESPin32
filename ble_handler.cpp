#include "ble_handler.h"
#include "motor_control.h"
#include "config.h"
#include "commands.h"
#include "telemetry.h"

BLEServer* pServer = NULL;
BLECharacteristic* pMotorCharacteristic = NULL;
BLECharacteristic* pTelemetryCharacteristic = NULL;
bool deviceConnected = false;

// Packet parser for BLE
PacketParser bleParser;

// Static buffers to avoid stack overflow in BLE callback task.
// Packet.payload is 1018 bytes and responseBuffer is 1024 bytes — putting
// both on the BLE task stack (~4KB total) reliably crashes the ESP32.
static Packet blePacket;
static uint8_t bleResponseBuffer[PACKET_MAX_LENGTH];

class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
    BLEDevice::startAdvertising();
  }
};

class MotorCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    
    if (value.length() > 0) {
      // Try packet protocol first
      bool packetReceived = false;
      
      for (size_t i = 0; i < value.length(); i++) {
        if (bleParser.processByte(value[i], blePacket)) {
          packetReceived = true;
          
          // Execute command
          size_t responseLength = 0;
          
          executeCommand(blePacket, bleResponseBuffer, &responseLength, true);
          
          // Send response via the mutex-protected helper to avoid
          // racing with the telemetry task's notify() call.
          if (responseLength > 0) {
            bleSendResponse(bleResponseBuffer, responseLength);
          }
        }
      }
      
      // Fall back to legacy command format if no packet detected
      if (!packetReceived && value.length() > 0) {
        char cmd = value[0];
        int val = 0;
        if (value.length() > 1) {
          val = atoi(value.substr(1).c_str());
        }
        
        Serial.print("Legacy command received: ");
        Serial.println(cmd);
        Serial.print("Value: ");
        Serial.println(val);
        
        handleMotorCommand(cmd, val);
      }
    }
  }
};

void initBLE() {
  Serial.println("Initializing Bluetooth...");
  BLEDevice::init("ESP32_Motor_Control");
  
  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  
  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Create Motor Control Characteristic (Write)
  pMotorCharacteristic = pService->createCharacteristic(
                    MOTOR_CHAR_UUID,
                    BLECharacteristic::PROPERTY_WRITE
                  );
  pMotorCharacteristic->setCallbacks(new MotorCallbacks());
  
  // Create Telemetry Characteristic (Read + Notify)
  pTelemetryCharacteristic = pService->createCharacteristic(
                    TELEMETRY_CHAR_UUID,
                    BLECharacteristic::PROPERTY_READ |
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
  pTelemetryCharacteristic->addDescriptor(new BLE2902());
  
  // Start service
  pService->start();
  
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("✓ BLE Server started");
  Serial.println("Device name: ESP32_Motor_Control");
}
