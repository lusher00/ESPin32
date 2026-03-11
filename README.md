# ESP32 Motor Controller v2.0

A robust motor control system with PID feedback, multi-interface command protocol, and encoder-based RPM measurement.

## Features

- **Unified Packet Protocol**: Commands work identically over UART, BLE, and WiFi
- **PID Speed Control**: Closed-loop RPM control with tunable parameters
- **Direct PWM Mode**: Bypass PID for manual motor control
- **POV Display**: Persistence of vision LED display with polar coordinate addressing
- **Sync Pulse Detection**: Accurate RPM measurement using custom encoder disk
- **Error Detection**: CRC-16/XMODEM checksums on all packets
- **Optional ACKs**: Choose between fire-and-forget or confirmed delivery
- **Configuration Storage**: Save/load settings to flash memory
- **LED Animations**: Visual feedback via NeoPixel strip
- **Telemetry Streaming**: Real-time status updates

## Hardware Setup

### Encoder Disk Geometry
- **1 post**: 20° wide
- **sync pulse falling edge**: LED bar @180° 
- **sync pulse rising edge**: LED bar @200°

### Pin Assignments
- **ENCODER_PIN** (A5): Encoder input (requires external 10kΩ pull-up to 3.3V)
- **MOTOR_PWM_PIN** (D3): Motor speed control (PWM)
- **MOTOR_DIR_PIN** (D4): Motor direction control
- **LED_PIN** (D13): NeoPixel data line
- **HEARTBEAT_PIN** (LED_BUILTIN): Status indicator

## Packet Protocol

### Structure
```
[START] [LENGTH] [COMMAND] [PAYLOAD...] [CRC16_H] [CRC16_L] [END]
  0xAA   1 byte    1 byte    0-1018 bytes  2 bytes          0x55
```

- **START_BYTE**: 0xAA
- **LENGTH**: Total packet length (6 to 1024 bytes)
- **COMMAND**: Command ID
- **PAYLOAD**: Command-specific data
- **CRC16**: CRC-16/XMODEM checksum (big-endian)
- **END_BYTE**: 0x55

### Command Reference

See `commands.h` for complete command definitions.

**Motor Control (0x10-0x1F)**
- `CMD_SET_TARGET_RPM` (0x10): Set target RPM
- `CMD_SET_DIRECTION` (0x11): Set motor direction
- `CMD_SET_MOTOR_ENABLE` (0x12): Enable/disable motor
- `CMD_SET_PWM_DIRECT` (0x13): Set PWM directly (bypasses PID)
- `CMD_EMERGENCY_STOP` (0x14): Emergency stop

**PID Control (0x20-0x2F)**
- `CMD_SET_PID_ENABLE` (0x20): Enable/disable PID
- `CMD_SET_KP` (0x21): Set proportional gain
- `CMD_SET_KI` (0x22): Set integral gain
- `CMD_SET_KD` (0x23): Set derivative gain
- `CMD_SET_PID_ALL` (0x24): Set all PID gains at once

**Configuration (0x30-0x3F)**
- `CMD_SAVE_CONFIG` (0x30): Save to flash
- `CMD_LOAD_CONFIG` (0x31): Load from flash
- `CMD_RESET_CONFIG` (0x32): Reset to defaults
- `CMD_SET_LED_PATTERN` (0x33): Set LED pattern

**Query (0x40-0x4F)**
- `CMD_GET_STATUS` (0x40): Get full status
- `CMD_GET_RPM` (0x41): Get current RPM
- `CMD_GET_PID_PARAMS` (0x42): Get PID gains
- `CMD_GET_ENCODER_COUNT` (0x43): Get encoder counts

## Usage Examples

### Python (UART)

```python
import serial
import struct
from packet_test import build_packet, send_command

ser = serial.Serial('COM3', 115200)

# Enable PID mode
send_command(ser, 0x20, struct.pack('B', 1))

# Set target RPM to 1000
send_command(ser, 0x10, struct.pack('<f', 1000.0))

# Enable motor
send_command(ser, 0x12, struct.pack('B', 1))
```

Run the test script:
```bash
python packet_test.py COM3
```

### Arduino/C++ (UART)

```cpp
#include "commands.h"

uint8_t buffer[32];
float rpm = 1000.0;

// Build packet
size_t len = buildPacket(buffer, CMD_SET_TARGET_RPM, 
                         (uint8_t*)&rpm, sizeof(float));

// Send over Serial
Serial.write(buffer, len);
```

### BLE

BLE supports both packet protocol and legacy ASCII commands:

**Packet mode** (recommended): Same binary packets as UART

**Legacy mode**: ASCII commands like `M1000`, `D1`, `E1`

## POV Display System

The system includes a Persistence of Vision (POV) display using the NeoPixel strip.

**How it works:**
- 34 LEDs mounted on a spinning arm
- Each LED traces a ring at a different radius
- Encoder synchronizes column updates with rotation angle
- Images stored as [34 columns × 34 LEDs × RGB]

**Frame Buffer:**
- Animations stored in flash memory
- Each frame = 3,468 bytes (34 columns × 34 LEDs × 3 bytes RGB)
- Multiple frames for animated content (e.g., clock with moving hands)

**Timing:**
- Updates triggered by encoder transitions (34 per revolution)
- Frame rate controlled by "revolutions per frame"
- Example: Clock with 60 frames at 1000 RPM = 1 frame per second

**POV Commands:**

```cpp
CMD_SELECT_ANIMATION (0x70)     // Select which animation to display
CMD_SET_FRAME_TIMING (0x71)     // Set revolutions per frame
CMD_SET_POV_ENABLE (0x74)       // Enable/disable POV display
CMD_UPLOAD_FRAME_START (0x76)   // Begin uploading animation
CMD_UPLOAD_FRAME_DATA (0x77)    // Upload column data
CMD_UPLOAD_FRAME_END (0x78)     // Finalize upload
```

**Creating POV Content:**

Use the included Python script to upload images:

```bash
# Upload a polar coordinate image
python pov_upload.py COM3 my_image.png

# Or upload the built-in test pattern
python pov_upload.py COM3
```

**Image Format:**
- Create images in polar coordinates
- Width = angular resolution (will be sampled to 34 columns)
- Height = 34 pixels (one per LED, inner to outer)
- Center of image = center of display

**Pre-loaded Animations:**
- Test pattern: Rainbow gradient (loaded at boot)
- Clock: TODO - implement clock face with moving hands

## Operating Modes

### PID Mode
1. Enable PID: `CMD_SET_PID_ENABLE` = 1
2. Set target RPM: `CMD_SET_TARGET_RPM`
3. Enable motor: `CMD_SET_MOTOR_ENABLE` = 1
4. Controller maintains target speed automatically

### Direct PWM Mode
1. Disable PID: `CMD_SET_PID_ENABLE` = 0
2. Set PWM value: `CMD_SET_PWM_DIRECT` (0-255)
3. Enable motor: `CMD_SET_MOTOR_ENABLE` = 1
4. Motor runs at fixed duty cycle

## Encoder Disk Pattern

```
Degree layout (one revolution):
  12° post, 8° void (×16) = 320°
  20° post, 20° void (×1) = 40°
  Total = 360°

Transitions: 34 per revolution
  - 32 from standard posts
  - 2 from wide post/void (sync marker)
```

Sync detection: Wide void (20°) is ~2.5× longer than normal void (8°)

## PID Tuning

Default values:
- **Kp = 2.0**
- **Ki = 0.5**
- **Kd = 0.1**

Tuning steps:
1. Set Ki=0, Kd=0
2. Increase Kp until oscillation
3. Set Kp to 50-60% of oscillation point
4. Increase Ki slowly to eliminate steady-state error
5. Add small Kd to reduce overshoot

## Installation

1. Install Arduino IDE with ESP32 support
2. Install libraries:
   - Adafruit NeoPixel
   - Preferences (built-in)
3. Upload to Arduino Nano ESP32
4. Connect hardware with 10kΩ pull-up on encoder pin

## Files

- `commands.h/cpp` - Packet protocol implementation
- `motor_control.h/cpp` - PID and motor control
- `pov_display.h/cpp` - POV display system
- `ble_handler.h/cpp` - Bluetooth interface
- `led_animation.h/cpp` - NeoPixel effects
- `telemetry.h/cpp` - Status reporting
- `config.h` - Pin definitions and constants
- `packet_test.py` - Python test utility
- `pov_upload.py` - POV animation uploader

## Troubleshooting

**No encoder pulses:**
- Check 10kΩ pull-up resistor to 3.3V
- Verify sensor alignment with disk
- Monitor ENCODER_PIN voltage (should toggle 0-3.3V)

**RPM inaccurate:**
- Verify disk has correct pattern (34 transitions/rev)
- Check sync pulse detection in Serial Monitor
- Adjust SYNC_THRESHOLD if needed

**CRC errors:**
- Confirm baud rate is 115200
- Check USB cable quality (Nano ESP32 connectors can be flaky)
- Ensure common ground

**PID unstable:**
- Reduce Kp gain
- Check mechanical binding
- Verify adequate power supply

## License

MIT License

## Author

Created February 2026
