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
  motor1.Enable();
  motor2.Enable();
  motor3.Enable();
  motor4.Enable();
}

// ============================================================
// Set speed for one motor
// Index: 1=FL, 2=FR, 3=RL, 4=RR
// ============================================================
void setMotorSpeed(int motorIndex, int spd) {
  BTS7960 *m;

  switch (motorIndex) {
    case 1: m = &motor1; break;
    case 2: m = &motor2; break;
    case 3: m = &motor3; break;
    case 4: m = &motor4; break;
    default: return;
  }

  int pwm = constrain(abs(spd), 0, 255);
  if (pwm > 0 && pwm < 80) pwm = 80;  // your deadzone fix

  if (abs(spd) < 5) {
    m->Stop();
  } 
  else if (spd > 5) {
    m->TurnRight(pwm);
  } 
  else if (spd < -5) {
    m->TurnLeft(pwm);
  }
}

// ============================================================
// Set all 4 motor speeds
// ============================================================
void setMotorSpeeds(int s1, int s2, int s3, int s4) {
  setMotorSpeed(1, s1);
  setMotorSpeed(2, s2);
  setMotorSpeed(3, s3);
  setMotorSpeed(4, s4);
}
