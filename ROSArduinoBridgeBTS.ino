/*********************************************************************
 *  ROSArduinoBridge - ESP32-S3 + BTS7960 4-Motor Version
 
    Enhanced with:
    - 4 independent motor control
    - Advanced PID with velocity filtering
    - EEPROM PID parameter storage
    - Improved encoder handling
    
    Based on the Pi Robot Project: http://www.pirobot.org
    
    Software License Agreement (BSD License)
    Copyright (c) 2012, Patrick Goebel.
 *********************************************************************/

#define USE_BASE      // Enable the base controller code

/* Serial port baud rate */
#define BAUDRATE     115200

/* Maximum PWM signal */
#define MAX_PWM        255

#include "Arduino.h"

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Variable initialization */

// A pair of variables to help parse serial commands
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first through fourth arguments
char argv1[16];
char argv2[16];
char argv3[16];
char argv4[16];

// The arguments converted to integers
long arg1;
long arg2;
long arg3;
long arg4;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg4 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  float pid_args[4];
  
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  arg3 = atoi(argv3);
  arg4 = atoi(argv4);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
    
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
    
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
    
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
    
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
    
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
    
  case PING:
    Serial.println(Ping(arg1));
    break;
    
#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(MOTOR_1));
    Serial.print(" ");
    Serial.print(readEncoder(MOTOR_2));
    Serial.print(" ");
    Serial.print(readEncoder(MOTOR_3));
    Serial.print(" ");
    Serial.println(readEncoder(MOTOR_4));
    break;
    
  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
    
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    
    // If all speeds are zero, stop and reset
    if (arg1 == 0 && arg2 == 0 && arg3 == 0 && arg4 == 0) {
      setMotorSpeeds(0, 0, 0, 0);
      resetPID();
      moving = 0;
    }
    else {
      moving = 1;
    }
    
    // Set target ticks per frame for each motor
    motor1PID.TargetTicksPerFrame = arg1;
    motor2PID.TargetTicksPerFrame = arg2;
    motor3PID.TargetTicksPerFrame = arg3;
    motor4PID.TargetTicksPerFrame = arg4;
    
    Serial.println("OK"); 
    break;
    
  case MOTOR_RAW_PWM:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    resetPID();
    moving = 0; // Disable PID temporarily
    setMotorSpeeds(arg1, arg2, arg3, arg4);
    Serial.println("OK"); 
    break;
    
  case UPDATE_PID:
    // Format: u motor:Kp:Ki:Kd
    // motor = 0-3 for motor1-4, or 9 for all motors
    i = 0;
    while ((str = strtok_r(p, ":", &p)) != '\0') {
      pid_args[i] = atof(str);
      i++;
      if (i >= 4) break;
    }
    
    if (i >= 4) {
      int motorIndex = (int)pid_args[0];
      
      if (motorIndex == 0 || motorIndex == 9) {
        motor1PID.Kp = pid_args[1];
        motor1PID.Ki = pid_args[2];
        motor1PID.Kd = pid_args[3];
      }
      if (motorIndex == 1 || motorIndex == 9) {
        motor2PID.Kp = pid_args[1];
        motor2PID.Ki = pid_args[2];
        motor2PID.Kd = pid_args[3];
      }
      if (motorIndex == 2 || motorIndex == 9) {
        motor3PID.Kp = pid_args[1];
        motor3PID.Ki = pid_args[2];
        motor3PID.Kd = pid_args[3];
      }
      if (motorIndex == 3 || motorIndex == 9) {
        motor4PID.Kp = pid_args[1];
        motor4PID.Ki = pid_args[2];
        motor4PID.Kd = pid_args[3];
      }
      
      Serial.println("OK");
    } else {
      Serial.println("Invalid PID format");
    }
    break;
    
  case SAVE_PID:
    savePID();
    Serial.println("OK");
    break;
    
  case LOAD_PID:
    loadPID();
    Serial.println("OK");
    break;
    
  case GET_PID:
    // Return PID parameters for specified motor (or all if arg1 == 9)
    if (arg1 == 0 || arg1 == 9) {
      Serial.print("M1: ");
      Serial.print(motor1PID.Kp); Serial.print(":");
      Serial.print(motor1PID.Ki); Serial.print(":");
      Serial.println(motor1PID.Kd);
    }

    if (arg1 == 1 || arg1 == 9) {
      Serial.print("M2: ");
      Serial.print(motor2PID.Kp); Serial.print(":");
      Serial.print(motor2PID.Ki); Serial.print(":");
      Serial.println(motor2PID.Kd);
    }
    if (arg1 == 2 || arg1 == 9) {
      Serial.print("M3: ");
      Serial.print(motor3PID.Kp); Serial.print(":");
      Serial.print(motor3PID.Ki); Serial.print(":");
      Serial.println(motor3PID.Kd);
    }
    if (arg1 == 3 || arg1 == 9) {
      Serial.print("M4: ");
      Serial.print(motor4PID.Kp); Serial.print(":");
      Serial.print(motor4PID.Ki); Serial.print(":");
      Serial.println(motor4PID.Kd);
    }
    break;
#endif

  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
  
  // Initialize the motor controller if used
#ifdef USE_BASE
  initEncoders();
  initMotorController();
  loadPID();  // Load PID parameters from EEPROM
  resetPID();
  
  Serial.println("ESP32-S3 4-Motor Controller Ready");
  Serial.print("Baudrate: ");
  Serial.println(BAUDRATE);
#endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      else if (arg == 3) argv3[index] = NULL;
      else if (arg == 4) argv4[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      else if (arg == 2) {
        argv2[index] = NULL;
        arg = 3;
        index = 0;
      }
      else if (arg == 3) {
        argv3[index] = NULL;
        arg = 4;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
      else if (arg == 3) {
        argv3[index] = chr;
        index++;
      }
      else if (arg == 4) {
        argv4[index] = chr;
        index++;
      }
    }
  }
  
  // If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0, 0, 0);
    moving = 0;
  }
#endif
}