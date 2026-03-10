#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================

// Compile-time debug level (set before compiling)
// 0 = No debug output (production)
// 1 = Errors only
// 2 = Warnings + Errors
// 3 = Info + Warnings + Errors
// 4 = Debug + Info + Warnings + Errors
// 5 = Verbose (everything including ISR events)

#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 5  // Default to Info level
#endif

// Runtime debug switches (can be toggled via commands)
struct DebugFlags {
  bool motor_control;      // Motor and PID debug
  bool encoder;            // Encoder transitions (NOT in ISR)
  bool pov_display;        // POV frame updates
  bool ble;                // BLE events
  bool commands;           // Command execution
  bool timing;             // Performance timing
  bool telemetry;          // Telemetry data
  bool sync_pulse;         // Sync pulse detection
};

extern DebugFlags debugFlags;

// ============================================================================
// DEBUG MACROS
// ============================================================================

// Error messages (Level 1)
#if DEBUG_LEVEL >= 1
  #define DEBUG_ERROR(msg) Serial.print("[ERROR] "); Serial.println(msg)
  #define DEBUG_ERRORF(fmt, ...) Serial.printf("[ERROR] " fmt "\n", ##__VA_ARGS__)
#else
  #define DEBUG_ERROR(msg)
  #define DEBUG_ERRORF(fmt, ...)
#endif

// Warning messages (Level 2)
#if DEBUG_LEVEL >= 2
  #define DEBUG_WARN(msg) Serial.print("[WARN] "); Serial.println(msg)
  #define DEBUG_WARNF(fmt, ...) Serial.printf("[WARN] " fmt "\n", ##__VA_ARGS__)
#else
  #define DEBUG_WARN(msg)
  #define DEBUG_WARNF(fmt, ...)
#endif

// Info messages (Level 3) - default
#if DEBUG_LEVEL >= 3
  #define DEBUG_INFO(msg) Serial.print("[INFO] "); Serial.println(msg)
  #define DEBUG_INFOF(fmt, ...) Serial.printf("[INFO] " fmt "\n", ##__VA_ARGS__)
#else
  #define DEBUG_INFO(msg)
  #define DEBUG_INFOF(fmt, ...)
#endif

// Debug messages (Level 4)
#if DEBUG_LEVEL >= 4
  #define DEBUG_DEBUG(msg) Serial.print("[DEBUG] "); Serial.println(msg)
  #define DEBUG_DEBUGF(fmt, ...) Serial.printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
  #define DEBUG_DEBUG(msg)
  #define DEBUG_DEBUGF(fmt, ...)
#endif

// Verbose messages (Level 5) - very detailed
#if DEBUG_LEVEL >= 5
  #define DEBUG_VERBOSE(msg) Serial.print("[VERBOSE] "); Serial.println(msg)
  #define DEBUG_VERBOSEF(fmt, ...) Serial.printf("[VERBOSE] " fmt "\n", ##__VA_ARGS__)
#else
  #define DEBUG_VERBOSE(msg)
  #define DEBUG_VERBOSEF(fmt, ...)
#endif

// ============================================================================
// CONDITIONAL DEBUG (runtime flags)
// ============================================================================

// Motor control debug
#define DEBUG_MOTOR(msg) if (debugFlags.motor_control) { DEBUG_DEBUG(msg); }
#define DEBUG_MOTORF(fmt, ...) if (debugFlags.motor_control) { DEBUG_DEBUGF(fmt, ##__VA_ARGS__); }

// Encoder debug (NOT called from ISR)
#define DEBUG_ENCODER(msg) if (debugFlags.encoder) { DEBUG_DEBUG(msg); }
#define DEBUG_ENCODERF(fmt, ...) if (debugFlags.encoder) { DEBUG_DEBUGF(fmt, ##__VA_ARGS__); }

// POV display debug
#define DEBUG_POV(msg) if (debugFlags.pov_display) { DEBUG_DEBUG(msg); }
#define DEBUG_POVF(fmt, ...) if (debugFlags.pov_display) { DEBUG_DEBUGF(fmt, ##__VA_ARGS__); }

// BLE debug
#define DEBUG_BLE(msg) if (debugFlags.ble) { DEBUG_DEBUG(msg); }
#define DEBUG_BLEF(fmt, ...) if (debugFlags.ble) { DEBUG_DEBUGF(fmt, ##__VA_ARGS__); }

// Command debug
#define DEBUG_CMD(msg) if (debugFlags.commands) { DEBUG_DEBUG(msg); }
#define DEBUG_CMDF(fmt, ...) if (debugFlags.commands) { DEBUG_DEBUGF(fmt, ##__VA_ARGS__); }

// Timing debug
#define DEBUG_TIMING(msg) if (debugFlags.timing) { DEBUG_DEBUG(msg); }
#define DEBUG_TIMINGF(fmt, ...) if (debugFlags.timing) { DEBUG_DEBUGF(fmt, ##__VA_ARGS__); }

// Sync pulse debug
#define DEBUG_SYNC(msg) if (debugFlags.sync_pulse) { DEBUG_DEBUG(msg); }
#define DEBUG_SYNCF(fmt, ...) if (debugFlags.sync_pulse) { DEBUG_DEBUGF(fmt, ##__VA_ARGS__); }

// ============================================================================
// ISR-SAFE DEBUGGING
// ============================================================================

// For ISR: increment counter, check in main loop
struct ISRDebugCounters {
  volatile uint32_t encoder_transitions;
  volatile uint32_t sync_pulses;
  volatile uint32_t debounce_rejects;
  volatile uint32_t pov_updates;
  volatile uint32_t last_pulse_width;
  volatile uint32_t last_avg_pulse_width;
};

extern ISRDebugCounters isrDebugCounters;

// Print ISR stats (call from main loop, not ISR!)
void printISRStats();
void resetISRStats();

// ============================================================================
// PERFORMANCE TIMING
// ============================================================================

class PerfTimer {
private:
  unsigned long startTime;
  const char* label;
  
public:
  PerfTimer(const char* lbl) : label(lbl) {
    if (debugFlags.timing) {
      startTime = micros();
    }
  }
  
  ~PerfTimer() {
    if (debugFlags.timing) {
      unsigned long elapsed = micros() - startTime;
      DEBUG_TIMINGF("%s took %lu us", label, elapsed);
    }
  }
};

// Use like: PERF_TIMER("Function name");
#define PERF_TIMER(name) PerfTimer _perf_timer(name)

// ============================================================================
// DEBUG CONTROL FUNCTIONS
// ============================================================================

void initDebug();
void setDebugFlag(const char* flag, bool enable);
void printDebugStatus();

#endif // DEBUG_H
