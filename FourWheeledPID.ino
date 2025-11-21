// =============================================
// ESP32-S3 + BTS7960 – 4 MOTOR PID CONTROL
// Clean Serial Plotter Version + Independent RPM Control
// PID1 tunable from Serial + EEPROM storage
// =============================================
#include <BTS7960.h>
#include <Arduino.h>
#include <EEPROM.h>

#define EEPROM_SIZE 64
#define KP_ADDR     0
#define KI_ADDR     4
#define KD_ADDR     8

// =====================
// MOTOR PIN ASSIGNMENTS
// =====================

// Motor 1 (FL)
#define RPWM1 5
#define LPWM1 4
#define ENCA1 15
#define ENCB1 16

// Motor 2 (FR)
#define RPWM2 47
#define LPWM2 21
#define ENCA2 2
#define ENCB2 1

// Motor 3 (RL)
#define RPWM3 9
#define LPWM3 10
#define ENCA3 17
#define ENCB3 18

// Motor 4 (RR)
#define RPWM4 42
#define LPWM4 41
#define ENCA4 3
#define ENCB4 8

// Motor drivers
BTS7960 motor1(99, 99, LPWM1, RPWM1);
BTS7960 motor2(99, 99, LPWM2, RPWM2);
BTS7960 motor3(99, 99, LPWM3, RPWM3);
BTS7960 motor4(99, 99, LPWM4, RPWM4);

// =====================
volatile long pos1=0, pos2=0, pos3=0, pos4=0;

portMUX_TYPE mux1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux2 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux3 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux4 = portMUX_INITIALIZER_UNLOCKED;

// =====================
float vFilt1=0, vFilt2=0, vFilt3=0, vFilt4=0;
float vPrev1=0, vPrev2=0, vPrev3=0, vPrev4=0;
float prevRaw1=0, prevRaw2=0, prevRaw3=0, prevRaw4=0;

float integral1=0, integral2=0, integral3=0, integral4=0;
float prevErr1=0, prevErr2=0, prevErr3=0, prevErr4=0;

const float CPR = 1808.81;
const float max_integral = 150.0;

long prevT = 0;

// ===============
float target1 = 0, target2 = 0, target3 = 0, target4 = 0;

// =====================
// PID constants (Motor 1 can be changed from Serial)
// =====================
float Kp1 = 30, Ki1 = 25, Kd1 = 0;      // Will be overwritten by EEPROM
float Kp2 = 30, Ki2 = 25, Kd2 = 0;
float Kp3 = 30, Ki3 = 25, Kd3 = 0;
float Kp4 = 30, Ki4 = 25, Kd4 = 0;

// =====================
// Encoder ISRs
// =====================
void IRAM_ATTR enc1(){ portENTER_CRITICAL_ISR(&mux1); pos1 += (digitalRead(ENCB1)?1:-1); portEXIT_CRITICAL_ISR(&mux1); }
void IRAM_ATTR enc2(){ portENTER_CRITICAL_ISR(&mux2); pos2 += (digitalRead(ENCB2)?1:-1); portEXIT_CRITICAL_ISR(&mux2); }
void IRAM_ATTR enc3(){ portENTER_CRITICAL_ISR(&mux3); pos3 += (digitalRead(ENCB3)?1:-1); portEXIT_CRITICAL_ISR(&mux3); }
void IRAM_ATTR enc4(){ portENTER_CRITICAL_ISR(&mux4); pos4 += (digitalRead(ENCB4)?1:-1); portEXIT_CRITICAL_ISR(&mux4); }

// =================================
//          EEPROM LOAD
// =================================
void loadPID() {
  EEPROM.begin(EEPROM_SIZE);

  float kp = EEPROM.readFloat(KP_ADDR);
  float ki = EEPROM.readFloat(KI_ADDR);
  float kd = EEPROM.readFloat(KD_ADDR);

  if (!isnan(kp)) Kp1 = kp;
  if (!isnan(ki)) Ki1 = ki;
  if (!isnan(kd)) Kd1 = kd;

  Serial.print("Loaded PID →  Kp="); Serial.print(Kp1);
  Serial.print("  Ki="); Serial.print(Ki1);
  Serial.print("  Kd="); Serial.println(Kd1);
}

// =================================
//          EEPROM SAVE
// =================================
void savePID() {
  EEPROM.writeFloat(KP_ADDR, Kp1);
  EEPROM.writeFloat(KI_ADDR, Ki1);
  EEPROM.writeFloat(KD_ADDR, Kd1);
  EEPROM.commit();

  Serial.println("PID Saved to EEPROM.");
}

void setup() {
  Serial.begin(115200);

  EEPROM.begin(EEPROM_SIZE);
  loadPID();

  motor1.Enable();
  motor2.Enable();
  motor3.Enable();
  motor4.Enable();

  pinMode(ENCA1, INPUT); pinMode(ENCB1, INPUT);
  pinMode(ENCA2, INPUT); pinMode(ENCB2, INPUT);
  pinMode(ENCA3, INPUT); pinMode(ENCB3, INPUT);
  pinMode(ENCA4, INPUT); pinMode(ENCB4, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA1), enc1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2), enc2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA3), enc3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA4), enc4, RISING);
}

float computePID(float target, float &vFilt, float raw, float &prevRaw,
                 float &integral, float &prevErr, float Kp, float Ki, float Kd, float dt) {

  vFilt = 0.854 * vFilt + 0.073 * raw + 0.073 * prevRaw;
  prevRaw = raw;

  float err = target - vFilt;
  integral += err * dt;
  integral = constrain(integral, -max_integral, max_integral);
  float der = (err - prevErr) / dt;
  prevErr = err;

  return Kp * err + Ki * integral + Kd * der;
}

void driveMotor(BTS7960 &m, float u) {
  int pwm = constrain(abs((int)u), 0, 255);
  if (pwm > 0 && pwm < 80) pwm = 80;

  if (abs(u) < 5) m.Stop();
  else if (u > 5) m.TurnRight(pwm);
  else if (u < -5) m.TurnLeft(pwm);
}

// =======================================================
//   SERIAL COMMANDS FOR PID1
// =======================================================
void handlePIDCommands(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.startsWith("KP=")) {
    Kp1 = cmd.substring(3).toFloat();
    savePID();
  }
  else if (cmd.startsWith("KI=")) {
    Ki1 = cmd.substring(3).toFloat();
    savePID();
  }
  else if (cmd.startsWith("KD=")) {
    Kd1 = cmd.substring(3).toFloat();
    savePID();
  }
  else if (cmd.startsWith("PID=")) {
    int c1 = cmd.indexOf(',');
    int c2 = cmd.lastIndexOf(',');

    Kp1 = cmd.substring(4, c1).toFloat();
    Ki1 = cmd.substring(c1+1, c2).toFloat();
    Kd1 = cmd.substring(c2+1).toFloat();

    savePID();
  }
}

void loop() {

  unsigned long now = micros();
  float dt = (now - prevT) / 1e6f;
  if (dt < 0.05) return;
  prevT = now;

  // =====================================================
  // ONLY MOTOR 1 GETS SINE-WAVE TARGET
  // =====================================================
  float t = millis() / 1000.0;
  const float minRPM = 10;
  const float maxRPM = 35;

  target1 = (maxRPM + minRPM)/2 + ((maxRPM - minRPM)/2) * sin(2 * PI * t / 2.0);

  // =====================================================
  // Serial input for PID tuning + motors 2,3,4 target
  // =====================================================
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');

    // PID commands for motor 1
    if (s.indexOf('=') > 0) {
      handlePIDCommands(s);
    }

    // Motor target commands (unchanged)
    s.toUpperCase();
    if (s.startsWith("M2")) { target2 = s.substring(2).toFloat(); integral2=prevErr2=0; }
    else if (s.startsWith("M3")) { target3 = s.substring(2).toFloat(); integral3=prevErr3=0; }
    else if (s.startsWith("M4")) { target4 = s.substring(2).toFloat(); integral4=prevErr4=0; }
    else if (s.startsWith("ALL")) {
      float v = s.substring(3).toFloat();
      target2 = target3 = target4 = v;
      integral2 = integral3 = integral4 = 0;
      prevErr2 = prevErr3 = prevErr4 = 0;
    }
  }

  // Copy encoder counts
  long p1,p2,p3,p4;
  portENTER_CRITICAL(&mux1); p1 = pos1; portEXIT_CRITICAL(&mux1);
  portENTER_CRITICAL(&mux2); p2 = pos2; portEXIT_CRITICAL(&mux2);
  portENTER_CRITICAL(&mux3); p3 = pos3; portEXIT_CRITICAL(&mux3);
  portENTER_CRITICAL(&mux4); p4 = pos4; portEXIT_CRITICAL(&mux4);

  float raw1 = (p1 - vPrev1) / dt / CPR * 60.0; vPrev1 = p1;
  float raw2 = (p2 - vPrev2) / dt / CPR * 60.0; vPrev2 = p2;
  float raw3 = (p3 - vPrev3) / dt / CPR * 60.0; vPrev3 = p3;
  float raw4 = (p4 - vPrev4) / dt / CPR * 60.0; vPrev4 = p4;

  float u1 = computePID(target1, vFilt1, raw1, prevRaw1, integral1, prevErr1, Kp1, Ki1, Kd1, dt);
  float u2 = computePID(target2, vFilt2, raw2, prevRaw2, integral2, prevErr2, Kp2, Ki2, Kd2, dt);
  float u3 = computePID(target3, vFilt3, raw3, prevRaw3, integral3, prevErr3, Kp3, Ki3, Kd3, dt);
  float u4 = computePID(target4, vFilt4, raw4, prevRaw4, integral4, prevErr4, Kp4, Ki4, Kd4, dt);

  driveMotor(motor1, u1);
  driveMotor(motor2, u2);
  driveMotor(motor3, u3);
  driveMotor(motor4, u4);

  // Serial Plotter Output
  Serial.print(target1); Serial.print(" ");
  Serial.print(vFilt1);  Serial.print(" ");
  Serial.print(target2); Serial.print(" ");
  Serial.print(vFilt2);  Serial.print(" ");
  Serial.print(target3); Serial.print(" ");
  Serial.print(vFilt3);  Serial.print(" ");
  Serial.print(target4); Serial.print(" ");
  Serial.println(vFilt4);
}