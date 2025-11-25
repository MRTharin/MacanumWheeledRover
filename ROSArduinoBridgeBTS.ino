/*********************************************************************
 *  ROSArduinoBridge - ESP32-S3 + BTS7960 4-Motor Version
 *  WITH DEBUG OUTPUT
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
int argIndex = 0;  // CHANGED from 'index' to 'argIndex'

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
  argIndex = 0;  // CHANGED
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
    
    Serial.print("DEBUG: Received speeds - M1:");
    Serial.print(arg1);
    Serial.print(" M2:");
    Serial.print(arg2);
    Serial.print(" M3:");
    Serial.print(arg3);
    Serial.print(" M4:");
    Serial.println(arg4);
    
    // If all speeds are zero, stop and reset
    if (arg1 == 0 && arg2 == 0 && arg3 == 0 && arg4 == 0) {
      setMotorSpeeds(0, 0, 0, 0);
      resetPID();
      moving = 0;
      Serial.println("DEBUG: All motors stopped");
    }
    else {
      moving = 1;
      Serial.println("DEBUG: Moving flag set to 1");
    }
    
    // Set target ticks per frame for each motor
    motor1PID.TargetTicksPerFrame = arg1;
    motor2PID.TargetTicksPerFrame = arg2;
    motor3PID.TargetTicksPerFrame = arg3;
    motor4PID.TargetTicksPerFrame = arg4;
    
    Serial.print("DEBUG: Targets set - M1:");
    Serial.print(motor1PID.TargetTicksPerFrame);
    Serial.print(" M2:");
    Serial.print(motor2PID.TargetTicksPerFrame);
    Serial.print(" M3:");
    Serial.print(motor3PID.TargetTicksPerFrame);
    Serial.print(" M4:");
    Serial.println(motor4PID.TargetTicksPerFrame);
    
    Serial.println("OK"); 
    break;
    
  case MOTOR_RAW_PWM:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    resetPID();
    moving = 0; // Disable PID temporarily
    
    Serial.print("DEBUG: Raw PWM - M1:");
    Serial.print(arg1);
    Serial.print(" M2:");
    Serial.print(arg2);
    Serial.print(" M3:");
    Serial.print(arg3);
    Serial.print(" M4:");
    Serial.println(arg4);
    
    setMotorSpeeds(arg1, arg2, arg3, arg4);
    Serial.println("OK"); 
    break;
    
  case UPDATE_PID:
    // Format: u motor:Kp:Ki:Kd
    // motor = 0-3 for motor1-4, or 9 for all motors
    i = 0;
    while ((str = strtok_r(p, ":", &p)) != NULL) {  // CHANGED '\0' to NULL
      pid_args[i] = atof(str);
      i++;
      if (i >= 4) break;
    }
    
    if (i >= 4) {
      int motorIndex = (int)pid_args[0];
      
      Serial.print("DEBUG: Updating PID for motor ");
      Serial.print(motorIndex);
      Serial.print(" - Kp:");
      Serial.print(pid_args[1]);
      Serial.print(" Ki:");
      Serial.print(pid_args[2]);
      Serial.print(" Kd:");
      Serial.println(pid_args[3]);
      
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
  delay(1000); // Give serial time to initialize
  
  Serial.println("\n\n=== ESP32-S3 4-Motor Controller ===");
  Serial.println("Initializing...");
  
  // Initialize the motor controller if used
#ifdef USE_BASE
  Serial.println("Initializing encoders...");
  initEncoders();
  
  Serial.println("Initializing motors...");
  initMotorController();
  
  Serial.println("Loading PID from EEPROM...");
  loadPID();
  
  Serial.println("Resetting PID...");
  resetPID();
  
  Serial.println("\n=== Initialization Complete ===");
  Serial.print("Baudrate: ");
  Serial.println(BAUDRATE);
  Serial.print("PID Rate: ");
  Serial.print(PID_RATE);
  Serial.println(" Hz");
  Serial.println("\nTest encoders by sending: e");
  Serial.println("Test motors with raw PWM: o 100 100 100 100");
  Serial.println("Move with PID: m 100 100 100 100");
  Serial.println("===========================\n");
#endif
}

// Debug counter
unsigned long debugCounter = 0;
unsigned long lastDebugTime = 0;

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
      if (arg == 1) argv1[argIndex] = NULL;  // CHANGED
      else if (arg == 2) argv2[argIndex] = NULL;  // CHANGED
      else if (arg == 3) argv3[argIndex] = NULL;  // CHANGED
      else if (arg == 4) argv4[argIndex] = NULL;  // CHANGED
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[argIndex] = NULL;  // CHANGED
        arg = 2;
        argIndex = 0;  // CHANGED
      }
      else if (arg == 2) {
        argv2[argIndex] = NULL;  // CHANGED
        arg = 3;
        argIndex = 0;  // CHANGED
      }
      else if (arg == 3) {
        argv3[argIndex] = NULL;  // CHANGED
        arg = 4;
        argIndex = 0;  // CHANGED
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        argv1[argIndex] = chr;  // CHANGED
        argIndex++;  // CHANGED
      }
      else if (arg == 2) {
        argv2[argIndex] = chr;  // CHANGED
        argIndex++;  // CHANGED
      }
      else if (arg == 3) {
        argv3[argIndex] = chr;  // CHANGED
        argIndex++;  // CHANGED
      }
      else if (arg == 4) {
        argv4[argIndex] = chr;  // CHANGED
        argIndex++;  // CHANGED
      }
    }
  }
  
  // If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
    debugCounter++;
    
    // Print debug info every 2 seconds if moving
    if (moving && (millis() - lastDebugTime > 2000)) {
      lastDebugTime = millis();
      Serial.print("DEBUG PID: Enc[");
      Serial.print(motor1PID.Encoder);
      Serial.print(",");
      Serial.print(motor2PID.Encoder);
      Serial.print(",");
      Serial.print(motor3PID.Encoder);
      Serial.print(",");
      Serial.print(motor4PID.Encoder);
      Serial.print("] Out[");
      Serial.print(motor1PID.output);
      Serial.print(",");
      Serial.print(motor2PID.output);
      Serial.print(",");
      Serial.print(motor3PID.output);
      Serial.print(",");
      Serial.print(motor4PID.output);
      Serial.print("] vFilt[");
      Serial.print(motor1PID.vFilt);
      Serial.print(",");
      Serial.print(motor2PID.vFilt);
      Serial.print(",");
      Serial.print(motor3PID.vFilt);
      Serial.print(",");
      Serial.print(motor4PID.vFilt);
      Serial.println("]");
    }
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0, 0, 0);
    moving = 0;
  }
#endif
}