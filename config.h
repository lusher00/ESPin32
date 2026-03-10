#ifndef CONFIG_H
#define CONFIG_H

// Pin Definitions
#define LED_PIN        D13      // NeoPixel data pin
#define LED_COUNT      34
#define HEARTBEAT_PIN  LED_BUILTIN

#define ENCODER_PIN    A7      // Interrupt pin for motor feedback
#define MOTOR_PWM_PIN  D3       // PWM for motor speed (H-bridge input 1)
#define MOTOR_DIR_PIN  D4       // Direction control (H-bridge input 2)

#define TEMP_SENSOR_PIN A0
#define DISTANCE_SENSOR_PIN A1

#define VALID_TRANSITIONS_PER_REV 1

// Timing Constants
const uint32_t HEARTBEAT_INTERVAL = 500;      // ms (blink rate)
const unsigned long RPM_INTERVAL = 100;       // Calculate RPM every 100ms
const unsigned long PID_INTERVAL = 50;        // Run PID every 50ms
const unsigned long TELEMETRY_INTERVAL = 100; // 100ms = 10Hz
const unsigned long ANIMATION_INTERVAL = 40;  // LED update rate

// Encoder Constants


const unsigned long DEBOUNCE_US = 1000;   // 5ms — single flag disk, ~60ms between pulses at 1000RPM


#define MOTOR_PWM_CHANNEL_A  0
#define MOTOR_PWM_CHANNEL_B  1
#define MOTOR_PWM_FREQ       20000  // 20kHz - above audible range
#define MOTOR_PWM_RES        8      // 8-bit resolution (0-255)

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define MOTOR_CHAR_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define TELEMETRY_CHAR_UUID "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"

#endif // CONFIG_H
