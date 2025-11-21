/* *************************************************************
   Encoder driver - ESP32-S3 4-encoder version
   ************************************************************ */

#ifdef USE_BASE

// =====================
// Encoder counters
// =====================
volatile long pos1 = 0;
volatile long pos2 = 0;
volatile long pos3 = 0;
volatile long pos4 = 0;

// =====================
// Mutex for each encoder
// =====================
portMUX_TYPE mux1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux2 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux3 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux4 = portMUX_INITIALIZER_UNLOCKED;

// =====================
// ISR FUNCTIONS
// =====================
void IRAM_ATTR encoderISR1() {
  portENTER_CRITICAL_ISR(&mux1);
  pos1 += (digitalRead(ENCB1) ? 1 : -1);
  portEXIT_CRITICAL_ISR(&mux1);
}

void IRAM_ATTR encoderISR2() {
  portENTER_CRITICAL_ISR(&mux2);
  pos2 += (digitalRead(ENCB2) ? 1 : -1);
  portEXIT_CRITICAL_ISR(&mux2);
}

void IRAM_ATTR encoderISR3() {
  portENTER_CRITICAL_ISR(&mux3);
  pos3 += (digitalRead(ENCB3) ? 1 : -1);
  portEXIT_CRITICAL_ISR(&mux3);
}

void IRAM_ATTR encoderISR4() {
  portENTER_CRITICAL_ISR(&mux4);
  pos4 += (digitalRead(ENCB4) ? 1 : -1);
  portEXIT_CRITICAL_ISR(&mux4);
}

// =====================
// Initialize all encoders
// =====================
void initEncoders() {
  pinMode(ENCA1, INPUT); pinMode(ENCB1, INPUT);
  pinMode(ENCA2, INPUT); pinMode(ENCB2, INPUT);
  pinMode(ENCA3, INPUT); pinMode(ENCB3, INPUT);
  pinMode(ENCA4, INPUT); pinMode(ENCB4, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA1), encoderISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2), encoderISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA3), encoderISR3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA4), encoderISR4, RISING);
}

// =====================
// ROS wrapper functions
// =====================
long readEncoder(int i) {
  long val = 0;

  switch (i) {
    case ENC_M1:
      portENTER_CRITICAL(&mux1);
      val = pos1;
      portEXIT_CRITICAL(&mux1);
      break;

    case ENC_M2:
      portENTER_CRITICAL(&mux2);
      val = pos2;
      portEXIT_CRITICAL(&mux2);
      break;

    case ENC_M3:
      portENTER_CRITICAL(&mux3);
      val = pos3;
      portEXIT_CRITICAL(&mux3);
      break;

    case ENC_M4:
      portENTER_CRITICAL(&mux4);
      val = pos4;
      portEXIT_CRITICAL(&mux4);
      break;
  }
  return val;
}

void resetEncoder(int i) {
  switch (i) {
    case ENC_M1:
      portENTER_CRITICAL(&mux1); pos1 = 0; portEXIT_CRITICAL(&mux1);
      break;

    case ENC_M2:
      portENTER_CRITICAL(&mux2); pos2 = 0; portEXIT_CRITICAL(&mux2);
      break;

    case ENC_M3:
      portENTER_CRITICAL(&mux3); pos3 = 0; portEXIT_CRITICAL(&mux3);
      break;

    case ENC_M4:
      portENTER_CRITICAL(&mux4); pos4 = 0; portEXIT_CRITICAL(&mux4);
      break;
  }
}

void resetEncoders() {
  resetEncoder(ENC_M1);
  resetEncoder(ENC_M2);
  resetEncoder(ENC_M3);
  resetEncoder(ENC_M4);
}

#endif
