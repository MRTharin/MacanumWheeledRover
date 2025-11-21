/***************************************************************
   Custom Motor Driver Header for ESP32-S3 + 4x BTS7960
   Matches 4-Motor PID Control Structure
***************************************************************/

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <BTS7960.h>

// Motor 1 (Front Left)
#define M1_LPWM 4
#define M1_RPWM 5

// Motor 2 (Front Right)
#define M2_LPWM 21
#define M2_RPWM 47

// Motor 3 (Rear Left)
#define M3_LPWM 10
#define M3_RPWM 9

// Motor 4 (Rear Right)
#define M4_LPWM 41
#define M4_RPWM 42

// Dummy EN pins (your setup ties EN to 3.3V)
#define EN_DUMMY 99

// BTS7960 Objects
extern BTS7960 motor1;
extern BTS7960 motor2;
extern BTS7960 motor3;
extern BTS7960 motor4;

// Function prototypes
void initMotorController();
void setMotorSpeed(int motorIndex, int speed);
void setMotorSpeeds(int m1, int m2, int m3, int m4);

#endif