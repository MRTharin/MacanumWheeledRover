#include "motor_driver.h"

// ============================================================
// Create 4 Motor Objects
// (Enable pins are dummy because EN tied to 3.3V)
// ============================================================
BTS7960 motor1(EN_DUMMY, EN_DUMMY, M1_LPWM, M1_RPWM);
BTS7960 motor2(EN_DUMMY, EN_DUMMY, M2_LPWM, M2_RPWM);
BTS7960 motor3(EN_DUMMY, EN_DUMMY, M3_LPWM, M3_RPWM);
BTS7960 motor4(EN_DUMMY, EN_DUMMY, M4_LPWM, M4_RPWM);

// ============================================================
// Initialize Motors
// ============================================================
void initMotorController() {
  // Enable all motors (library will handle the dummy EN pins gracefully)
  // motor1.Enable();
  // motor2.Enable();
  // motor3.Enable();
  // motor4.Enable();
  
  Serial.println("Motor controller initialized with BTS7960 library");
}

// ============================================================
// Set speed for one motor
// Index: 0=M1(FL), 1=M2(FR), 2=M3(RL), 3=M4(RR)
// ============================================================
void setMotorSpeed(int motorIndex, int spd) {
  BTS7960 *m;
  
  switch (motorIndex) {
    case 0: m = &motor1; break;  // Motor 1 (Front Left)
    case 1: m = &motor2; break;  // Motor 2 (Front Right)
    case 2: m = &motor3; break;  // Motor 3 (Rear Left)
    case 3: m = &motor4; break;  // Motor 4 (Rear Right)
    default: 
      Serial.print("ERROR: Invalid motor index: ");
      Serial.println(motorIndex);
      return;
  }
  
  // Apply deadzone and constrain PWM
  int pwm = constrain(abs(spd), 0, 255);
  if (pwm > 0 && pwm < 80) pwm = 80;  // Deadzone fix
  
  // Set motor direction and speed
  if (abs(spd) < 5) {
    m->Stop();
  } 
  else if (spd > 5) {
    m->TurnRight(pwm);  // Forward
  } 
  else { // spd < -5
    m->TurnLeft(pwm);   // Reverse
  }
}

// ============================================================
// Set all 4 motor speeds
// ============================================================
void setMotorSpeeds(int s1, int s2, int s3, int s4) {
  setMotorSpeed(0, s1);  // Motor 1
  setMotorSpeed(1, s2);  // Motor 2
  setMotorSpeed(2, s3);  // Motor 3
  setMotorSpeed(3, s4);  // Motor 4
}