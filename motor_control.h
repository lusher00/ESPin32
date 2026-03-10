#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <Preferences.h>

// External variables accessible to other modules
extern float targetRPM;
extern float currentRPM;
extern float pwmOutput;
extern bool motorDirection;
extern bool motorEnabled;

// PID gains
extern float Kp;
extern float Ki;
extern float Kd;

// Encoder variables
extern volatile unsigned long encoderCount;
extern volatile unsigned long validPulseCount;
extern volatile unsigned long avgPulseWidth;
extern volatile unsigned long avgVoidWidth;
extern bool pidEnabled;
extern bool directPWMMode;

// Function prototypes
void initMotorControl();
void startPOVTimer(unsigned long periodUs);
void stopPOVTimer();
void handleMotorCommand(char cmd, int value);  // Legacy - for BLE backward compatibility
void calculateRPMTask();
void pidTask();
void heartbeatTask();
void saveConfig();
void loadConfig();
void resetConfig();

// New packet-based API functions
void setTargetRPM(float rpm);
void setMotorDirection(bool forward);
void setMotorEnable(bool enable);
void setDirectPWM(uint8_t pwm);
void emergencyStop();
void setPIDEnable(bool enable);
void setKp(float kp);
void setKi(float ki);
void setKd(float kd);
void setPIDParams(float kp, float ki, float kd);

// Query functions
float getCurrentRPM();
void getPIDParams(float& kp, float& ki, float& kd);
void getEncoderCounts(uint32_t& total, uint32_t& valid);
void getStatusData(struct StatusData& status);

// Interrupt handler
void IRAM_ATTR encoderISR();

#endif // MOTOR_CONTROL_H

// POV show flag — set by ISR, cleared by main loop
extern volatile bool povNeedsShow;
