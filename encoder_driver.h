/* *************************************************************
   Encoder driver function definitions - ESP32-S3 version
   ************************************************************ */

#pragma once

// Handle up to 4 motors
#define ENC_M1 0
#define ENC_M2 1
#define ENC_M3 2
#define ENC_M4 3

// Define pins from your main code
#define ENCA1 15
#define ENCB1 16

#define ENCA2 2
#define ENCB2 1

#define ENCA3 17
#define ENCB3 18

#define ENCA4 3
#define ENCB4 8

long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
