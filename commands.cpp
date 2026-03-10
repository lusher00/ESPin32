#include "commands.h"
#include "motor_control.h"
#include "led_animation.h"
#include "pov_display.h"
#include "debug.h"

// ============================================================================
// CRC-16/XMODEM IMPLEMENTATION
// ============================================================================

uint16_t calculateCRC16(const uint8_t* data, size_t length) {
  uint16_t crc = 0x0000;
  
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = crc << 1;
      }
    }
  }
  
  return crc;
}

// ============================================================================
// PACKET BUILDING
// ============================================================================

size_t buildPacket(uint8_t* buffer, uint8_t command, const uint8_t* payload, uint8_t payloadLength) {
  if (payloadLength > PACKET_MAX_PAYLOAD) {
    return 0;
  }
  
  size_t packetLength = 6 + payloadLength; // start + length + cmd + payload + crc16 + end
  
  if (packetLength > PACKET_MAX_LENGTH) {
    return 0;
  }
  
  size_t idx = 0;
  
  // Start byte
  buffer[idx++] = PACKET_START_BYTE;
  
  // Length (total packet length)
  buffer[idx++] = (uint8_t)packetLength;
  
  // Command
  buffer[idx++] = command;
  
  // Payload
  if (payload && payloadLength > 0) {
    memcpy(&buffer[idx], payload, payloadLength);
    idx += payloadLength;
  }
  
  // Calculate CRC over length, command, and payload
  uint16_t crc = calculateCRC16(&buffer[1], idx - 1);
  
  // CRC (big-endian)
  buffer[idx++] = (uint8_t)(crc >> 8);
  buffer[idx++] = (uint8_t)(crc & 0xFF);
  
  // End byte
  buffer[idx++] = PACKET_END_BYTE;
  
  return idx;
}

// ============================================================================
// RESPONSE PACKET BUILDERS
// ============================================================================

size_t buildAckPacket(uint8_t* buffer, uint8_t originalCommand) {
  return buildPacket(buffer, CMD_ACK, &originalCommand, 1);
}

size_t buildNackPacket(uint8_t* buffer, uint8_t originalCommand, uint8_t errorCode) {
  uint8_t payload[2] = {originalCommand, errorCode};
  return buildPacket(buffer, CMD_NACK, payload, 2);
}

size_t buildStatusPacket(uint8_t* buffer, const StatusData& status) {
  return buildPacket(buffer, CMD_STATUS_RESPONSE, (const uint8_t*)&status, sizeof(StatusData));
}

size_t buildRPMPacket(uint8_t* buffer, float rpm) {
  return buildPacket(buffer, CMD_RPM_RESPONSE, (const uint8_t*)&rpm, sizeof(float));
}

size_t buildPIDPacket(uint8_t* buffer, float kp, float ki, float kd) {
  uint8_t payload[12];
  memcpy(&payload[0], &kp, 4);
  memcpy(&payload[4], &ki, 4);
  memcpy(&payload[8], &kd, 4);
  return buildPacket(buffer, CMD_PID_RESPONSE, payload, 12);
}

size_t buildEncoderPacket(uint8_t* buffer, uint32_t totalCount, uint32_t validCount) {
  uint8_t payload[8];
  memcpy(&payload[0], &totalCount, 4);
  memcpy(&payload[4], &validCount, 4);
  return buildPacket(buffer, CMD_ENCODER_RESPONSE, payload, 8);
}

// ============================================================================
// COMMAND EXECUTION
// ============================================================================

bool executeCommand(const Packet& packet, uint8_t* responseBuffer, size_t* responseLength, bool sendAck) {
  *responseLength = 0;
  
  switch (packet.command) {
    
    // ========================================================================
    // MOTOR CONTROL COMMANDS
    // ========================================================================
    
    case CMD_SET_TARGET_RPM: {
      if (packet.payloadLength != 4) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_LENGTH);
        }
        return false;
      }
      
      float rpm;
      memcpy(&rpm, packet.payload, 4);
      
      if (rpm < 0 || rpm > 1500) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_OUT_OF_RANGE);
        }
        return false;
      }
      
      setTargetRPM(rpm);
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_SET_DIRECTION: {
      if (packet.payloadLength != 1) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_LENGTH);
        }
        return false;
      }
      
      bool direction = (packet.payload[0] != 0);
      setMotorDirection(direction);
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_SET_MOTOR_ENABLE: {
      if (packet.payloadLength != 1) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_LENGTH);
        }
        return false;
      }
      
      bool enable = (packet.payload[0] != 0);
      setMotorEnable(enable);
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_SET_PWM_DIRECT: {
      if (packet.payloadLength != 1) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_LENGTH);
        }
        return false;
      }
      
      uint8_t pwm = packet.payload[0];
      setDirectPWM(pwm);
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_EMERGENCY_STOP: {
      emergencyStop();
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    // ========================================================================
    // PID CONTROL COMMANDS
    // ========================================================================
    
    case CMD_SET_PID_ENABLE: {
      if (packet.payloadLength != 1) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_LENGTH);
        }
        return false;
      }
      
      bool enable = (packet.payload[0] != 0);
      setPIDEnable(enable);
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_SET_KP: {
      if (packet.payloadLength != 4) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_LENGTH);
        }
        return false;
      }
      
      float kp;
      memcpy(&kp, packet.payload, 4);
      setKp(kp);
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_SET_KI: {
      if (packet.payloadLength != 4) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_LENGTH);
        }
        return false;
      }
      
      float ki;
      memcpy(&ki, packet.payload, 4);
      setKi(ki);
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_SET_KD: {
      if (packet.payloadLength != 4) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_LENGTH);
        }
        return false;
      }
      
      float kd;
      memcpy(&kd, packet.payload, 4);
      setKd(kd);
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_SET_PID_ALL: {
      if (packet.payloadLength != 12) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_LENGTH);
        }
        return false;
      }
      
      float kp, ki, kd;
      memcpy(&kp, &packet.payload[0], 4);
      memcpy(&ki, &packet.payload[4], 4);
      memcpy(&kd, &packet.payload[8], 4);
      
      setPIDParams(kp, ki, kd);
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    // ========================================================================
    // CONFIGURATION COMMANDS
    // ========================================================================
    
    case CMD_SAVE_CONFIG: {
      saveConfig();
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_LOAD_CONFIG: {
      loadConfig();
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_RESET_CONFIG: {
      resetConfig();
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_SET_LED_PATTERN: {
      if (packet.payloadLength != 1) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_LENGTH);
        }
        return false;
      }
      
      uint8_t pattern = packet.payload[0];
      if (pattern > 6) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_OUT_OF_RANGE);
        }
        return false;
      }
      
      setLEDPattern(pattern);
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    // ========================================================================
    // QUERY COMMANDS
    // ========================================================================
    
    case CMD_GET_STATUS: {
      StatusData status;
      getStatusData(status);
      *responseLength = buildStatusPacket(responseBuffer, status);
      return true;
    }
    
    case CMD_GET_RPM: {
      float rpm = getCurrentRPM();
      *responseLength = buildRPMPacket(responseBuffer, rpm);
      return true;
    }
    
    case CMD_GET_PID_PARAMS: {
      float kp, ki, kd;
      getPIDParams(kp, ki, kd);
      *responseLength = buildPIDPacket(responseBuffer, kp, ki, kd);
      return true;
    }
    
    case CMD_GET_ENCODER_COUNT: {
      uint32_t total, valid;
      getEncoderCounts(total, valid);
      *responseLength = buildEncoderPacket(responseBuffer, total, valid);
      return true;
    }
    
    // ========================================================================
    // POV DISPLAY COMMANDS
    // ========================================================================
    
    case CMD_SELECT_ANIMATION: {
      if (packet.payloadLength != 1) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_LENGTH);
        }
        return false;
      }
      
      uint8_t animId = packet.payload[0];
      selectAnimation(animId);
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_SET_FRAME_TIMING: {
      if (packet.payloadLength != 2) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_LENGTH);
        }
        return false;
      }
      
      uint16_t revPerFrame;
      memcpy(&revPerFrame, packet.payload, 2);
      setFrameTiming(revPerFrame);
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_GET_REVOLUTION_COUNT: {
      uint32_t revCount = getRevolutionCount();
      *responseLength = buildPacket(responseBuffer, CMD_REVOLUTION_RESPONSE, 
                                    (uint8_t*)&revCount, sizeof(uint32_t));
      return true;
    }
    
    case CMD_RESET_REVOLUTION: {
      resetRevolutionCount();
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_SET_POV_ENABLE: {
      Serial.printf("[CMD] POV_ENABLE received, payload[0]=%d\n", packet.payload[0]);
      if (packet.payloadLength != 1) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_LENGTH);
        }
        return false;
      }
      
      bool enable = (packet.payload[0] != 0);
      setPOVEnable(enable);
      
      // In setPOVEnable:
      Serial.printf("[POV] enable set to %d\n", enable);
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_GET_CURRENT_FRAME: {
      uint16_t frameNum = getCurrentFrame();
      *responseLength = buildPacket(responseBuffer, CMD_FRAME_RESPONSE, 
                                    (uint8_t*)&frameNum, sizeof(uint16_t));
      return true;
    }
    
    case CMD_UPLOAD_FRAME_START: {
      if (packet.payloadLength != 3) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_LENGTH);
        }
        return false;
      }
      
      uint8_t animId = packet.payload[0];
      uint16_t totalFrames;
      memcpy(&totalFrames, &packet.payload[1], 2);
      
      if (!startAnimationUpload(animId, totalFrames)) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_OUT_OF_RANGE);
        }
        return false;
      }
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_UPLOAD_FRAME_DATA: {
      // Payload: uint16 frame_num + uint8 column + 102 bytes RGB data
      if (packet.payloadLength != (2 + 1 + POV_LEDS * 3)) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_LENGTH);
        }
        return false;
      }
      
      uint16_t frameNum;
      memcpy(&frameNum, &packet.payload[0], 2);
      uint8_t column = packet.payload[2];
      const uint8_t* rgbData = &packet.payload[3];
      
      if (!uploadFrameData(frameNum, column, rgbData)) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_OUT_OF_RANGE);
        }
        return false;
      }
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_UPLOAD_FRAME_END: {
      if (!endAnimationUpload()) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_COMMAND);
        }
        return false;
      }
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    // ========================================================================
    // DEBUG COMMANDS
    // ========================================================================
    
    case CMD_SET_DEBUG_FLAG: {
      if (packet.payloadLength != 2) {
        if (sendAck) {
          *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_LENGTH);
        }
        return false;
      }
      
      uint8_t flagId = packet.payload[0];
      bool enable = (packet.payload[1] != 0);
      
      switch (flagId) {
        case DEBUG_FLAG_MOTOR: setDebugFlag("motor", enable); break;
        case DEBUG_FLAG_ENCODER: setDebugFlag("encoder", enable); break;
        case DEBUG_FLAG_POV: setDebugFlag("pov", enable); break;
        case DEBUG_FLAG_BLE: setDebugFlag("ble", enable); break;
        case DEBUG_FLAG_COMMANDS: setDebugFlag("commands", enable); break;
        case DEBUG_FLAG_TIMING: setDebugFlag("timing", enable); break;
        case DEBUG_FLAG_TELEMETRY: setDebugFlag("telemetry", enable); break;
        case DEBUG_FLAG_SYNC: setDebugFlag("sync", enable); break;
        case DEBUG_FLAG_ALL: setDebugFlag("all", enable); break;
        default:
          if (sendAck) {
            *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_OUT_OF_RANGE);
          }
          return false;
      }
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    case CMD_GET_ISR_STATS: {
      // Snapshot volatile ISR counters into locals first — memcpy can't
      // accept volatile pointers directly.
      uint32_t s0 = isrDebugCounters.encoder_transitions;
      uint32_t s1 = isrDebugCounters.sync_pulses;
      uint32_t s2 = isrDebugCounters.debounce_rejects;
      uint32_t s3 = isrDebugCounters.pov_updates;
      uint32_t s4 = isrDebugCounters.last_pulse_width;
      uint32_t s5 = isrDebugCounters.last_avg_pulse_width;

      uint8_t stats[24];
      memcpy(&stats[0],  &s0, 4);
      memcpy(&stats[4],  &s1, 4);
      memcpy(&stats[8],  &s2, 4);
      memcpy(&stats[12], &s3, 4);
      memcpy(&stats[16], &s4, 4);
      memcpy(&stats[20], &s5, 4);
      
      *responseLength = buildPacket(responseBuffer, CMD_ISR_STATS_RESPONSE, stats, 24);
      return true;
    }
    
    case CMD_RESET_ISR_STATS: {
      resetISRStats();
      
      if (sendAck) {
        *responseLength = buildAckPacket(responseBuffer, packet.command);
      }
      return true;
    }
    
    // ========================================================================
    // UNKNOWN COMMAND
    // ========================================================================
    
    default: {
      if (sendAck) {
        *responseLength = buildNackPacket(responseBuffer, packet.command, ERR_INVALID_COMMAND);
      }
      return false;
    }
  }
}

// ============================================================================
// PACKET PARSER IMPLEMENTATION
// ============================================================================

PacketParser::PacketParser() {
  reset();
}

void PacketParser::reset() {
  state = WAITING_FOR_START;
  bufferIndex = 0;
  packetLength = 0;
  command = 0;
  receivedCRC = 0;
}

bool PacketParser::processByte(uint8_t byte, Packet& packet) {
  switch (state) {
    
    case WAITING_FOR_START:
      if (byte == PACKET_START_BYTE) {
        buffer[0] = byte;
        bufferIndex = 1;
        state = READING_LENGTH;
      }
      break;
      
    case READING_LENGTH:
      packetLength = byte;
      
      if (packetLength < PACKET_MIN_LENGTH || packetLength > PACKET_MAX_LENGTH) {
        reset();
        break;
      }
      
      buffer[bufferIndex++] = byte;
      state = READING_COMMAND;
      break;
      
    case READING_COMMAND:
      command = byte;
      buffer[bufferIndex++] = byte;
      
      // Calculate payload length
      // Total length = start(1) + length(1) + command(1) + payload(N) + crc(2) + end(1)
      packet.payloadLength = packetLength - 6;
      
      if (packet.payloadLength > 0) {
        state = READING_PAYLOAD;
      } else {
        state = READING_CRC_HIGH;
      }
      break;
      
    case READING_PAYLOAD:
      buffer[bufferIndex++] = byte;
      
      // Check if we've read all payload bytes
      if (bufferIndex >= (3 + packet.payloadLength)) {
        state = READING_CRC_HIGH;
      }
      break;
      
    case READING_CRC_HIGH:
      receivedCRC = (uint16_t)byte << 8;
      state = READING_CRC_LOW;
      break;
      
    case READING_CRC_LOW:
      receivedCRC |= byte;
      state = READING_END;
      break;
      
    case READING_END:
      if (byte != PACKET_END_BYTE) {
        reset();
        break;
      }
      
      // Verify CRC (calculate over length, command, and payload)
      uint16_t calculatedCRC = calculateCRC16(&buffer[1], bufferIndex - 1);
      
      if (calculatedCRC != receivedCRC) {
        reset();
        break;
      }
      
      // Valid packet received!
      packet.command = command;
      
      if (packet.payloadLength > 0) {
        memcpy(packet.payload, &buffer[3], packet.payloadLength);
      }
      
      reset();
      return true;
  }
  
  return false;
}
