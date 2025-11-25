/* Functions and type-defs for PID control with advanced filtering and EEPROM storage.
   Enhanced version with independent motor control and improved PID algorithm.
*/

#ifndef DIFF_CONTROLLER_H
#define DIFF_CONTROLLER_H

#include <EEPROM.h>

// EEPROM Configuration
#define EEPROM_SIZE 64
#define KP_ADDR     0
#define KI_ADDR     4
#define KD_ADDR     8
#define KP2_ADDR    12
#define KI2_ADDR    16
#define KD2_ADDR    20
#define KP3_ADDR    24
#define KI3_ADDR    28
#define KD3_ADDR    32
#define KP4_ADDR    36
#define KI4_ADDR    40
#define KD4_ADDR    44

// Motor constants
const float CPR = 1808.81;  // Counts per revolution
const float MAX_INTEGRAL = 150.0;

/* PID setpoint info For a Motor with advanced filtering */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count
  
  // Advanced PID variables
  float vFilt;                   // filtered velocity
  float vPrev;                   // previous velocity for calculation
  float prevRaw;                 // previous raw velocity for filtering
  float integral;                // integrated error
  float prevErr;                 // previous error
  
  // PID gains
  float Kp;
  float Ki;
  float Kd;
  
  long output;                   // last motor setting
}
SetPointInfo;

SetPointInfo motor1PID, motor2PID, motor3PID, motor4PID;

unsigned char moving = 0; // is the base in motion?

// Timing variables
unsigned long prevPIDTime = 0;

/*
 * Load PID parameters from EEPROM
 */
void loadPID() {
  EEPROM.begin(EEPROM_SIZE);
  
  // Motor 1
  float kp = EEPROM.readFloat(KP_ADDR);
  float ki = EEPROM.readFloat(KI_ADDR);
  float kd = EEPROM.readFloat(KD_ADDR);
  
  if (!isnan(kp)) motor1PID.Kp = kp; else motor1PID.Kp = 30.0;
  if (!isnan(ki)) motor1PID.Ki = ki; else motor1PID.Ki = 25.0;
  if (!isnan(kd)) motor1PID.Kd = kd; else motor1PID.Kd = 0.0;
  
  // Motor 2
  kp = EEPROM.readFloat(KP2_ADDR);
  ki = EEPROM.readFloat(KI2_ADDR);
  kd = EEPROM.readFloat(KD2_ADDR);
  
  if (!isnan(kp)) motor2PID.Kp = kp; else motor2PID.Kp = 30.0;
  if (!isnan(ki)) motor2PID.Ki = ki; else motor2PID.Ki = 25.0;
  if (!isnan(kd)) motor2PID.Kd = kd; else motor2PID.Kd = 0.0;
  
  // Motor 3
  kp = EEPROM.readFloat(KP3_ADDR);
  ki = EEPROM.readFloat(KI3_ADDR);
  kd = EEPROM.readFloat(KD3_ADDR);
  
  if (!isnan(kp)) motor3PID.Kp = kp; else motor3PID.Kp = 30.0;
  if (!isnan(ki)) motor3PID.Ki = ki; else motor3PID.Ki = 25.0;
  if (!isnan(kd)) motor3PID.Kd = kd; else motor3PID.Kd = 0.0;
  
  // Motor 4
  kp = EEPROM.readFloat(KP4_ADDR);
  ki = EEPROM.readFloat(KI4_ADDR);
  kd = EEPROM.readFloat(KD4_ADDR);
  
  if (!isnan(kp)) motor4PID.Kp = kp; else motor4PID.Kp = 30.0;
  if (!isnan(ki)) motor4PID.Ki = ki; else motor4PID.Ki = 25.0;
  if (!isnan(kd)) motor4PID.Kd = kd; else motor4PID.Kd = 0.0;
  
  Serial.println("PID Parameters Loaded from EEPROM");
}

/*
 * Save PID parameters to EEPROM
 */
void savePID() {
  EEPROM.writeFloat(KP_ADDR, motor1PID.Kp);
  EEPROM.writeFloat(KI_ADDR, motor1PID.Ki);
  EEPROM.writeFloat(KD_ADDR, motor1PID.Kd);
  
  EEPROM.writeFloat(KP2_ADDR, motor2PID.Kp);
  EEPROM.writeFloat(KI2_ADDR, motor2PID.Ki);
  EEPROM.writeFloat(KD2_ADDR, motor2PID.Kd);
  
  EEPROM.writeFloat(KP3_ADDR, motor3PID.Kp);
  EEPROM.writeFloat(KI3_ADDR, motor3PID.Ki);
  EEPROM.writeFloat(KD3_ADDR, motor3PID.Kd);
  
  EEPROM.writeFloat(KP4_ADDR, motor4PID.Kp);
  EEPROM.writeFloat(KI4_ADDR, motor4PID.Ki);
  EEPROM.writeFloat(KD4_ADDR, motor4PID.Kd);
  
  EEPROM.commit();
  
  Serial.println("PID Parameters Saved to EEPROM");
}

/*
 * Initialize PID variables to zero to prevent startup spikes
 */
void resetPID() {
  // Motor 1
  motor1PID.TargetTicksPerFrame = 0.0;
  motor1PID.Encoder = readEncoder(0);
  motor1PID.PrevEnc = motor1PID.Encoder;
  motor1PID.output = 0;
  motor1PID.vFilt = 0;
  motor1PID.vPrev = 0;
  motor1PID.prevRaw = 0;
  motor1PID.integral = 0;
  motor1PID.prevErr = 0;
  
  // Motor 2
  motor2PID.TargetTicksPerFrame = 0.0;
  motor2PID.Encoder = readEncoder(1);
  motor2PID.PrevEnc = motor2PID.Encoder;
  motor2PID.output = 0;
  motor2PID.vFilt = 0;
  motor2PID.vPrev = 0;
  motor2PID.prevRaw = 0;
  motor2PID.integral = 0;
  motor2PID.prevErr = 0;
  
  // Motor 3
  motor3PID.TargetTicksPerFrame = 0.0;
  motor3PID.Encoder = readEncoder(2);
  motor3PID.PrevEnc = motor3PID.Encoder;
  motor3PID.output = 0;
  motor3PID.vFilt = 0;
  motor3PID.vPrev = 0;
  motor3PID.prevRaw = 0;
  motor3PID.integral = 0;
  motor3PID.prevErr = 0;
  
  // Motor 4
  motor4PID.TargetTicksPerFrame = 0.0;
  motor4PID.Encoder = readEncoder(3);
  motor4PID.PrevEnc = motor4PID.Encoder;
  motor4PID.output = 0;
  motor4PID.vFilt = 0;
  motor4PID.vPrev = 0;
  motor4PID.prevRaw = 0;
  motor4PID.integral = 0;
  motor4PID.prevErr = 0;
}

/* 
 * Compute PID with advanced filtering
 */
float computePID(SetPointInfo * p, float dt) {
  // Calculate raw velocity from encoder change
  float raw = (p->Encoder - p->PrevEnc) / dt / CPR * 60.0;
  p->PrevEnc = p->Encoder;
  
  // Apply low-pass filter for smoother velocity estimation
  p->vFilt = 0.854 * p->vFilt + 0.073 * raw + 0.073 * p->prevRaw;
  p->prevRaw = raw;
  
  // Convert target from ticks per frame to RPM
  // Assuming PID runs at 30Hz, so dt ≈ 0.033s per frame
  float targetRPM = (p->TargetTicksPerFrame / CPR) * (60.0 / dt);
  
  // Calculate error
  float err = targetRPM - p->vFilt;
  
  // Integral with anti-windup
  p->integral += err * dt;
  p->integral = constrain(p->integral, -MAX_INTEGRAL, MAX_INTEGRAL);
  
  // Derivative
  float der = (err - p->prevErr) / dt;
  p->prevErr = err;
  
  // PID output
  float output = p->Kp * err + p->Ki * p->integral + p->Kd * der;
  
  return output;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p, float dt) {
  float output = computePID(p, dt);
  
  // Convert to PWM range
  p->output = constrain((long)output, -MAX_PWM, MAX_PWM);
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  // Calculate time delta
  unsigned long now = micros();
  float dt = (now - prevPIDTime) / 1e6f;
  
  // Don't run if dt is too small (less than 20ms)
  if (dt < 0.02) return;
  
  prevPIDTime = now;
  
  /* Read the encoders */
  motor1PID.Encoder = readEncoder(0);
  motor2PID.Encoder = readEncoder(1);
  motor3PID.Encoder = readEncoder(2);
  motor4PID.Encoder = readEncoder(3);
  
  /* If we're not moving there is nothing more to do */
  if (!moving) {
    // Reset PIDs once to prevent startup spikes
    if (motor1PID.prevErr != 0 || motor2PID.prevErr != 0 || 
        motor3PID.prevErr != 0 || motor4PID.prevErr != 0) {
      resetPID();
    }
    return;
  }
  
  /* Compute PID update for each motor */
  doPID(&motor1PID, dt);
  doPID(&motor2PID, dt);
  doPID(&motor3PID, dt);
  doPID(&motor4PID, dt);
  
  /* Set the motor speeds accordingly */
  setMotorSpeeds(motor1PID.output, motor2PID.output, motor3PID.output, motor4PID.output);
}

#endif