/*
 * ESP32 Motor Controller v2.0
 * 
 * Features:
 * - Unified packet-based command protocol (BLE, WiFi, UART)
 * - PID speed control with encoder feedback
 * - Direct PWM control mode
 * - Sync pulse detection (34 transitions/rev)
 * - NeoPixel animation
 * - Telemetry streaming
 * - Flash-based configuration storage
 * 
 * Encoder disk: 16 posts (12°) + 1 wide post (20°) + voids (16×8° + 1×20°)
 * 
 * Author: Claude + User
 * Date: 2026
 */

#include "config.h"
#include "motor_control.h"
#include "ble_handler.h"
#include "led_animation.h"
#include "telemetry.h"
#include "commands.h"
#include "pov_display.h"
#include "debug.h"


// Packet parser for Serial
PacketParser serialParser;

// ISR stats print interval
unsigned long lastISRStatsPrint = 0;
const unsigned long ISR_STATS_INTERVAL = 5000;  // Print every 5 seconds if encoder debug enabled

void setup() {
  Serial.begin(115200);
  
  // Wait for serial connection and print boot message
  delay(2000);
  Serial.println("\n\n=================================");
  Serial.println("ESP32 Motor Controller v2.0");
  Serial.println("=================================");
  Serial.println("Booting...");
  
  // Initialize debug system first
  initDebug();
  
  // Initialize all subsystems
  initMotorControl();
  initBLE();
  initLEDs();
  initTelemetry();
  initPOV();

  // Load default animations — heap diagnostics to catch malloc failures
  Serial.printf("[INFO] Free heap before animations: %u bytes\n", ESP.getFreeHeap());
  loadTestAnimation();
  Serial.printf("[INFO] Free heap after rainbow:  %u bytes\n", ESP.getFreeHeap());
  loadSpokesAnimation();
  Serial.printf("[INFO] Free heap after spokes:   %u bytes\n", ESP.getFreeHeap());
  loadBullseyeAnimation();
  Serial.printf("[INFO] Free heap after bullseye: %u bytes\n", ESP.getFreeHeap());
  loadSpiralAnimation();
  Serial.printf("[INFO] Free heap after spiral:   %u bytes\n", ESP.getFreeHeap());
  loadClockAnimation();
  Serial.printf("[INFO] Free heap after clock:    %u bytes\n", ESP.getFreeHeap());
  
  Serial.println("\n=================================");
  Serial.println("READY - Waiting for connections");
  Serial.println("=================================\n");
  
  DEBUG_INFO("Type 'help' for debug commands");
}

void loop() {
  heartbeatTask();
  // Don't run the normal LED animation while POV is active —
  // they share the same strip and animationTask calls strip.show() every 40ms
  // which would block interrupts and fight with the POV column updates.
  if (!getPOVEnable()) {
    animationTask();
  }
  sendTelemetryTask();
  calculateRPMTask();
  pidTask();
  handleSerialPackets();
  handleSerialCommands();
  
  if (povNeedsShow) {
    povNeedsShow = false;
    strip.show();
  }

  // Print ISR stats periodically if encoder debug enabled
  if (debugFlags.encoder && (millis() - lastISRStatsPrint >= ISR_STATS_INTERVAL)) {
    printISRStats();
    lastISRStatsPrint = millis();
  }

  static unsigned long lastPovDebug = 0;
  if (millis() - lastPovDebug > 1000) {
    lastPovDebug = millis();
    Serial.printf("[POV] enabled:%d col:%d frame:%d rev:%d\n",
      getPOVEnable(),
      povState.currentColumn,
      povState.currentFrame,
      povState.revolutionCount);
  }
}

void handleSerialCommands() {
  // Handle ASCII debug commands (non-packet mode)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.length() == 0) return;
    
    // Don't process if it looks like binary data
    if (cmd[0] == 0xAA) return;
    
    if (cmd == "help") {
      Serial.println("\n=== Debug Commands ===");
      Serial.println("debug <flag> on/off  - Toggle debug flags");
      Serial.println("  Flags: motor, encoder, pov, ble, commands, timing, sync, all");
      Serial.println("stats                - Print ISR statistics");
      Serial.println("reset_stats          - Reset ISR statistics");
      Serial.println("status               - Print debug status");
      Serial.println("help                 - Show this help");
      Serial.println("======================\n");
    }
    else if (cmd == "stats") {
      printISRStats();
    }
    else if (cmd == "reset_stats") {
      resetISRStats();
    }
    else if (cmd == "status") {
      printDebugStatus();
    }
    else if (cmd.startsWith("debug ")) {
      // Parse: debug <flag> <on|off>
      int firstSpace = cmd.indexOf(' ');
      int secondSpace = cmd.indexOf(' ', firstSpace + 1);
      
      if (secondSpace > 0) {
        String flag = cmd.substring(firstSpace + 1, secondSpace);
        String state = cmd.substring(secondSpace + 1);
        
        bool enable = (state == "on" || state == "1");
        setDebugFlag(flag.c_str(), enable);
      }
    }
  }
}

void povDisplayTask() {
  // Display POV column (non-blocking, called frequently)
  // The encoder ISR updates the column index, we just display it
  if (getPOVEnable()) {
    displayPOVColumn();
  }
}

void handleSerialPackets() {
  static Packet serialPacket;
  static uint8_t serialResponseBuffer[PACKET_MAX_LENGTH];
  
  while (Serial.available()) {
    uint8_t byte = Serial.read();
    
    if (serialParser.processByte(byte, serialPacket)) {
      // Valid packet received
      size_t responseLength = 0;
      
      // Execute command with ACK enabled
      executeCommand(serialPacket, serialResponseBuffer, &responseLength, true);
      
      // Send response if generated
      if (responseLength > 0) {
        Serial.write(serialResponseBuffer, responseLength);
      }
    }
  }
}
