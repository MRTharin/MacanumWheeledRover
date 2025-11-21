/* Define single-letter commands that will be sent by the PC over the
   serial link. Extended with new commands for 4-motor control.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define MOTOR_RAW_PWM  'o'
#define PING           'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
#define SAVE_PID       'y'      // Save PID parameters to EEPROM
#define LOAD_PID       'z'      // Load PID parameters from EEPROM
#define GET_PID        'g'      // Get current PID parameters

// Motor indices for 4-motor system
#define MOTOR_1        0  // Front Left
#define MOTOR_2        1  // Front Right
#define MOTOR_3        2  // Rear Left
#define MOTOR_4        3  // Rear Right

// Legacy compatibility
#define LEFT           0
#define RIGHT          1

#endif