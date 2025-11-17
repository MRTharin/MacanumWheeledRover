/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
#endif

#ifdef BTS7960_MOTOR_DRIVER
  // Left motor pins
  #define LEFT_MOTOR_L_EN 25
  #define LEFT_MOTOR_R_EN 26
  #define LEFT_MOTOR_L_PWM 32
  #define LEFT_MOTOR_R_PWM 33
  
  // Right motor pins  
  #define RIGHT_MOTOR_L_EN 27
  #define RIGHT_MOTOR_R_EN 28
  #define RIGHT_MOTOR_L_PWM 34
  #define RIGHT_MOTOR_R_PWM 35
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
