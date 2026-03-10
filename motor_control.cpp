#include "motor_control.h"
#include "config.h"
#include "led_animation.h"
#include "commands.h"
#include "pov_display.h"
#include "debug.h"

// Global variables
Preferences prefs;

float targetRPM = 0;
float currentRPM = 0;
float pwmOutput = 0;
bool motorDirection = true;
bool motorEnabled = false;

// PID gains
float Kp = 2.0;
float Ki = 0.5;
float Kd = 0.1;

// PID state
float errorSum = 0;
float lastError = 0;
unsigned long lastPIDUpdate = 0;

// Encoder/Interrupt variables
volatile unsigned long encoderCount = 0;
volatile unsigned long validPulseCount = 0;
volatile unsigned long lastInterruptTime = 0;
volatile unsigned long pulseWidth = 0;
volatile bool syncDetected = false;
bool rpmFresh = false;
volatile unsigned long syncPulseWidth = 0;
volatile unsigned long avgPulseWidth = 0;    // Average post width
volatile unsigned long avgVoidWidth = 0;     // Average void width (for sync detection)
volatile unsigned long allTransitionAvg = 0; // Average of ALL transitions (for RPM)
volatile unsigned long avgRisingWidth = 0;
volatile unsigned long avgFallingWidth = 0;
volatile unsigned long lastRisingTime = 0;
volatile unsigned long lastFallingTime = 0;
bool pidEnabled = false;
bool directPWMMode = false;   // When true, pidTask() will not touch the PWM pin

// RPM calculation
unsigned long lastRPMCalc = 0;
unsigned long lastEncoderCount = 0;

// Heartbeat
unsigned long lastBeat = 0;
bool beatState = false;

// ── POV COLUMN TIMER ──
// Hardware timer fires every (revolution_period / POV_COLUMNS) to advance column.
hw_timer_t* povTimer = nullptr;
volatile bool povTimerRunning = false;

volatile bool povNeedsShow = false;

void IRAM_ATTR povTimerISR() {
  if (!getPOVEnable()) return;
  if (allTransitionAvg == 0) return;
  povState.currentColumn = (povState.currentColumn + 1) % POV_COLUMNS;
  displayPOVColumn();
  povNeedsShow = true;  // signal main loop
}


void startPOVTimer(unsigned long periodUs) {
  // periodUs is the full revolution period in microseconds.
  // Reject implausibly short periods (< 10ms = > 6000 RPM) or zero.
  if (periodUs == 0 || periodUs < 10000) {
    Serial.printf("[POV] startPOVTimer rejected bad period: %lu\n", periodUs);
    return;
  }
  
  uint64_t columnUs = periodUs / POV_COLUMNS;
  // Per-column minimum: 200µs = ~5000 columns/sec. Protects timer hardware.
  if (columnUs < 200) columnUs = 200;

  if (povTimer == nullptr) {
    povTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(povTimer, &povTimerISR, true);
  }
  timerAlarmDisable(povTimer);        // stop before reconfiguring
  timerAlarmWrite(povTimer, columnUs, true);
  timerAlarmEnable(povTimer);
  povTimerRunning = true;
}

void stopPOVTimer() {
  if (povTimer != nullptr) {
    timerAlarmDisable(povTimer);
    timerDetachInterrupt(povTimer);
    timerEnd(povTimer);
    povTimer = nullptr;             // force full reinit on next start
  }
  povTimerRunning = false;
}

void IRAM_ATTR encoderISR() {
  unsigned long currentTime = micros();

  // Debounce — single flag disk, pulses should be ~60ms apart at 1000RPM
  // Use a generous debounce to reject noise but not real pulses
  if (currentTime - lastInterruptTime < 5000) { // 5ms minimum between pulses
    isrDebugCounters.debounce_rejects++;
    return;
  }

  // Measure revolution period
  unsigned long period = currentTime - lastInterruptTime;
  lastInterruptTime = currentTime;
  encoderCount++;
  validPulseCount++;
  isrDebugCounters.encoder_transitions++;

  // Smooth period with EMA (16-sample)
  if (allTransitionAvg == 0) allTransitionAvg = period;
  else allTransitionAvg = (allTransitionAvg * 15 + period) / 16;

  // Update RPM
  // period is microseconds per revolution
  // RPM = 60,000,000 / period
  isrDebugCounters.last_pulse_width = period;

  // Reset POV to column 0 (sync) and restart column timer with new period
  if (getPOVEnable()) {
    povEncoderUpdate();  // resets column 0, advances frame

    // Restart timer with updated period.
    // If the timer was torn down (stopPOVTimer nulls povTimer), recreate it.
    uint64_t columnUs = allTransitionAvg / POV_COLUMNS;
    if (columnUs < 500) columnUs = 500;
    if (povTimer == nullptr) {
      povTimer = timerBegin(0, 80, true);
      timerAttachInterrupt(povTimer, &povTimerISR, true);
      timerAlarmWrite(povTimer, columnUs, true);
      timerAlarmEnable(povTimer);
      povTimerRunning = true;
    } else {
      timerAlarmWrite(povTimer, columnUs, true);
    }
  }
}

// Write PWM to the correct H-bridge leg based on direction.
// DRV8871: IN1 and IN2 are independent direction+PWM inputs.
//   Forward fast decay: IN1=PWM, IN2=LOW
//   Reverse fast decay: IN1=LOW, IN2=PWM
//   Coast:              IN1=LOW, IN2=LOW
//   Brake:              IN1=HIGH, IN2=HIGH (avoid)
static void applyMotorPWM(int pwm) {
  pwm = constrain(pwm, 0, 255);
  if (pwm == 0) {
    ledcWrite(MOTOR_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_PWM_CHANNEL_B, 0);
    return;
  }
  if (motorDirection) {
    ledcWrite(MOTOR_PWM_CHANNEL_A, pwm);
    ledcWrite(MOTOR_PWM_CHANNEL_B, 0);
  } else {
    ledcWrite(MOTOR_PWM_CHANNEL_A, 0);
    ledcWrite(MOTOR_PWM_CHANNEL_B, pwm);
  }
}

void initMotorControl() {
  // Load saved configuration
  loadConfig();
  
  // Both H-bridge legs are outputs. Start coasting (both LOW).
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_PWM_PIN, LOW);
  digitalWrite(MOTOR_DIR_PIN, LOW);

  // Configure LEDC for both H-bridge pins at 20kHz
  ledcSetup(MOTOR_PWM_CHANNEL_A, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcSetup(MOTOR_PWM_CHANNEL_B, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(MOTOR_PWM_PIN, MOTOR_PWM_CHANNEL_A);
  ledcAttachPin(MOTOR_DIR_PIN, MOTOR_PWM_CHANNEL_B);

  DEBUG_INFO("✓ Motor pins configured");
  
  // Configure encoder pin with pullup and interrupt
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
  DEBUG_INFOF("✓ Encoder interrupt attached (Pin %d)", ENCODER_PIN);
  
  // Configure heartbeat
  pinMode(HEARTBEAT_PIN, OUTPUT);
  DEBUG_INFO("✓ Heartbeat configured");
}

void handleMotorCommand(char cmd, int value) {
  switch(cmd) {
    case 'M': // Set target RPM (0-1500)
      targetRPM = constrain(value, 0, 1500);
      DEBUG_MOTORF("Target RPM set to: %.1f", targetRPM);
      
      saveConfig();
      break;
      
    case 'D': // Set direction (0=backward, 1=forward)
      motorDirection = (value != 0);
      applyMotorPWM(0);  // Re-latch direction pins at zero speed
      DEBUG_MOTORF("Direction: %s", motorDirection ? "FORWARD" : "BACKWARD");
      saveConfig();
      break;
      
    case 'E': // Enable/disable motor (0=off, 1=on)
      motorEnabled = (value != 0);
      DEBUG_MOTOR(motorEnabled ? "Motor ENABLED" : "Motor DISABLED");
      break;
      
    case 'P': // Set Kp gain (value / 10)
      Kp = value / 10.0;
      DEBUG_MOTORF("Kp set to: %.2f", Kp);
      
      saveConfig();
      break;
      
    case 'I': // Set Ki gain (value / 100)
      Ki = value / 100.0;
      DEBUG_MOTORF("Ki set to: %.3f", Ki);
      
      saveConfig();
      break;
      
    case 'K': // Set Kd gain (value / 100)
      Kd = value / 100.0;
      DEBUG_MOTORF("Kd set to: %.3f", Kd);
      
      saveConfig();
      break;
      
    case 'L': // Set LED pattern (0-6)
      setLEDPattern(constrain(value, 0, 6));
      DEBUG_INFOF("LED pattern set to: %d", value);
      
      break;
      
    case 'C': // Save current config
      saveConfig();
      DEBUG_INFO("✓ Config saved");
      break;
      
    case 'R': // Reset to defaults
      resetConfig();
      DEBUG_INFO("✓ Config reset to defaults");
      break;
      
    default:
      DEBUG_WARN("Unknown command");
      break;
  }
}

void saveConfig() {
  prefs.begin("motor", false);
  
  prefs.putFloat("Kp", Kp);
  prefs.putFloat("Ki", Ki);
  prefs.putFloat("Kd", Kd);
  prefs.putFloat("targetRPM", targetRPM);
  prefs.putBool("motorDir", motorDirection);
  
  prefs.end();
  
  DEBUG_INFO("✓ Config saved to flash");
}

void loadConfig() {
  prefs.begin("motor", true);
  
  Kp = prefs.getFloat("Kp", 1.1);
  Ki = prefs.getFloat("Ki", 0.46);
  Kd = prefs.getFloat("Kd", 0.2);
  targetRPM = prefs.getFloat("targetRPM", 0.0);
  motorDirection = prefs.getBool("motorDir", true);
  
  prefs.end();
  
  DEBUG_INFO("✓ Config loaded from flash");
}

void resetConfig() {
  Kp = 1.1;
  Ki = 0.46;
  Kd = 0.2;
  targetRPM = 0.0;
  motorDirection = true;
  
  saveConfig();
}

void calculateRPMTask() {
  if (millis() - lastRPMCalc >= RPM_INTERVAL) {
    noInterrupts();
    unsigned long avgTrans = allTransitionAvg;
    unsigned long snapshot = validPulseCount;
    interrupts();

    // RPM from average inter-transition time — uses all 34 transitions per rev.
    // Smoother than post-only because it has 2x the sample rate.
    if (avgTrans > 0) {
      currentRPM = 60000000.0f / (float)avgTrans;  // single pulse per revolution
    } else {
      currentRPM = 0;
    }

    // Zero out if no pulses arrived this interval (motor stopped)
    unsigned long pulses = snapshot - lastEncoderCount;
    lastEncoderCount = snapshot;
    if (pulses == 0) currentRPM = 0;

    lastRPMCalc = millis();
    rpmFresh = true;   // Signal pidTask that a new reading is ready

    // Auto-enable PID once encoder is stable
    if (!pidEnabled && !directPWMMode && snapshot > 20 && avgTrans > 0 && targetRPM > 0) {
      pidEnabled = true;
      errorSum = 0;
      lastError = 0;
      DEBUG_INFO("PID auto-enabled after stable encoder readings");
    }
  }
}

void pidTask() {
  if (millis() - lastPIDUpdate >= PID_INTERVAL) {

    // Direct PWM mode: PID doesn't touch the pin, but motor kill / E-stop still work
    if (directPWMMode) {
      if (!motorEnabled) applyMotorPWM(0);
      lastPIDUpdate = millis();
      return;
    }

    // Only run PID if enabled AND motor is enabled
    if (!pidEnabled || !motorEnabled) {
      errorSum = 0;
      lastError = 0;
      pwmOutput = 0;
      applyMotorPWM(0);
      lastPIDUpdate = millis();
      return;
    }

    // Only update PID when a fresh RPM reading is available
    // to avoid accumulating integral on stale data
    if (!rpmFresh) {
      lastPIDUpdate = millis();
      return;
    }
    rpmFresh = false;

    float error = targetRPM - currentRPM;

    // Proportional
    float P = Kp * error;

    // Integral with anti-windup
    errorSum += error * (RPM_INTERVAL / 1000.0f);
    errorSum = constrain(errorSum, -500, 500);
    float I = Ki * errorSum;

    // Derivative - scaled by interval in seconds
    float D = Kd * (error - lastError) / (RPM_INTERVAL / 1000.0f);
    lastError = error;

    pwmOutput = P + I + D;
    pwmOutput = constrain(pwmOutput, 0, 255);

    applyMotorPWM((int)pwmOutput);
    lastPIDUpdate = millis();
  }
}

void heartbeatTask() {
  if (millis() - lastBeat >= HEARTBEAT_INTERVAL) {
    lastBeat = millis();
    beatState = !beatState;
    digitalWrite(HEARTBEAT_PIN, beatState);
  }
}

// ============================================================================
// NEW PACKET-BASED API FUNCTIONS
// ============================================================================

void setTargetRPM(float rpm) {
  targetRPM = constrain(rpm, 0, 1500);
  DEBUG_MOTORF("Target RPM set to: %.1f", targetRPM);
  
  saveConfig();
}

void setMotorDirection(bool forward) {
  motorDirection = forward;
  applyMotorPWM(0);  // Re-latch direction pins at zero speed
  DEBUG_MOTORF("Direction: %s", motorDirection ? "FORWARD" : "BACKWARD");
  saveConfig();
}

void setMotorEnable(bool enable) {
  motorEnabled = enable;
  DEBUG_MOTOR(motorEnabled ? "Motor ENABLED" : "Motor DISABLED");
}

void setDirectPWM(uint8_t pwm) {
  directPWMMode = true;
  pidEnabled = false;
  applyMotorPWM(pwm);
  pwmOutput = pwm;
  DEBUG_MOTORF("Direct PWM set to: %d", pwm);
}

void emergencyStop() {
  motorEnabled = false;
  pidEnabled = false;
  directPWMMode = false;
  applyMotorPWM(0);
  pwmOutput = 0;
  DEBUG_ERROR("EMERGENCY STOP");
}

void setPIDEnable(bool enable) {
  if (enable) {
    // Switching to PID mode: leave direct PWM, reset integrator
    directPWMMode = false;
    pidEnabled = true;
    errorSum = 0;
    lastError = 0;
    DEBUG_MOTOR("PID ENABLED");
  } else {
    // Switching to Direct PWM mode: stop PID output, arm directPWMMode
    // so pidTask() won't zero the pin when the slider is moved.
    pidEnabled = false;
    directPWMMode = true;
    applyMotorPWM(0);
    pwmOutput = 0;
    DEBUG_MOTOR("PID DISABLED (direct PWM mode)");
  }
}

void setKp(float kp) {
  Kp = kp;
  DEBUG_MOTORF("Kp set to: %.2f", Kp);
  saveConfig();
}

void setKi(float ki) {
  Ki = ki;
  DEBUG_MOTORF("Ki set to: %.3f", Ki);
  saveConfig();
}

void setKd(float kd) {
  Kd = kd;
  DEBUG_MOTORF("Kd set to: %.3f", Kd);
  saveConfig();
}

void setPIDParams(float kp, float ki, float kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  DEBUG_MOTORF("PID params set - Kp: %.2f, Ki: %.3f, Kd: %.3f", Kp, Ki, Kd);
  saveConfig();
}

float getCurrentRPM() {
  return currentRPM;
}

void getPIDParams(float& kp, float& ki, float& kd) {
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

void getEncoderCounts(uint32_t& total, uint32_t& valid) {
  total = encoderCount;
  valid = validPulseCount;
}

void getStatusData(StatusData& status) {
  status.currentRPM = currentRPM;
  status.targetRPM = targetRPM;
  status.pwmOutput = pwmOutput;
  status.motorEnabled = motorEnabled ? 1 : 0;
  status.motorDirection = motorDirection ? 1 : 0;
  status.pidEnabled = pidEnabled ? 1 : 0;
  status.encoderCount = encoderCount;
  status.validPulseCount = validPulseCount;
  status.Kp = Kp;
  status.Ki = Ki;
  status.Kd = Kd;
}

