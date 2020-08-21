#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H


#define LINO_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors

#define USE_L298_DRIVER


#define DEBUG 1

#define K_P 0.05 // P constant
#define K_I 0.9 // I constant
#define K_D 0.1 // D constant

//define your robot' specs here
#define MAX_RPM 3000              // motor's maximum RPM
#define COUNTS_PER_REV 1550       // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.154      // wheel's diameter in meters
#define PWM_BITS 10                // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.235  // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.38  // distance between front and back wheels. Ignore this if you're on 2WD/ACKERMANN

//=================BIGGER ROBOT SPEC (BTS7960)=============================
// #define K_P 0.05  // P constant
// #define K_I 0.9   // I constant
// #define K_D 0.1   // D constant

// define your robot' specs here
// #define MAX_RPM 45               // motor's maximum RPM
// #define COUNTS_PER_REV 4000      // wheel encoder's no of ticks per rev
// #define WHEEL_DIAMETER 0.15      // wheel's diameter in meters
// #define PWM_BITS 8               // PWM Resolution of the microcontroller
// #define LR_WHEELS_DISTANCE 0.32  // distance between left and right wheels
// #define FR_WHEELS_DISTANCE 0.38  // distance between front and back wheels. Ignore this if you're on 2WD/ACKERMANN
//================= END OF BIGGER ROBOT SPEC =============================

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

/// ENCODER PINS
#define MOTOR1_ENCODER_A 6

#define MOTOR2_ENCODER_A 20

//MOTOR PINS
#define MOTOR_DRIVER L298

#define MOTOR1_PWM 5
#define MOTOR1_IN_A 2
#define MOTOR1_IN_B 11

#define MOTOR2_PWM 23
#define MOTOR2_IN_A 17
#define MOTOR2_IN_B 16


#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX


#endif

