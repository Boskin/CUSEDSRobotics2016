#ifndef CONSTANTS_H
#define CONSTANTS_H

const int DRIVE_TOWARD_GOAL = 0;
const int AVOID_OBSTACLE_LEFT = 1;
const int AVOID_OBSTACLE_RIGHT = 2;
const int AVOID_OBSTACLE_BACK = 3;
const int PROCEED_LEFT = 4;
const int PROCEED_RIGHT = 5;
const int ALIGN_WITH_GOAL = 6;

const float ANGLE_EQUALITY_TOLERANCE = 10.0f;

// These are all subject to change
const struct UltrasonicPort {
  int trig;
  int echo;
} ULTRASONIC_PORTS[] = {
  {42, 43}, // Trig and echo for left sensor
  {46, 47}, // Trig and echo for center sensor
  {50, 51} // Trig and echo for right sensor
};

// Factor to multiply duration by to get distance
const float DURATION_DISTANCE_FACTOR = 1 / 58.2;

// In cm
const float OBJECT_TOO_CLOSE = 80.0f;

const struct CompassPort {
  int sda;
  int sdl;
} COMPASS_PORTS[] = {
  {20, 21}
};

const int COMPASS_I2C_ADDRESS = 0x1E;

/* const struct MotorControllerPorts {
  int pwma;
  int ain1;
  int ain2;
  int pwmb;
  int bin1;
  int bin2;
  int stby;
} MOTOR_CONTROLLER_PORTS[] = {
  // Motor controller 0
  {2, 28, 29, // PWMA, AIN1, AIN2
   3, 30, 31, // PWMB, BIN1, BIN2
   GLOBAL_STANDBY}, // Standby
  {4, 32, 33,
   5, 34, 35,
   GLOBAL_STANDBY},
  {6, 36, 37,
   7, 38, 39,
   GLOBAL_STANDBY}
}; */

const struct MotorControllerPorts {
  int pwm;
  int in1;
  int in2;
} MOTOR_CONTROLLER_PORTS[] = {
  {2, 28, 29}, // Rear left
  {3, 30, 31}, // Front right
  {4, 32, 33}, // Front left
  {5, 44, 45}, // Rear right
  {6, 36, 37}, // Mid left
  {7, 38, 39} // Mid right
};

const int DIR_RIGHT = 0;
const int DIR_LEFT = 1;

// All motors will share the same standby port
const int GLOBAL_STANDBY = 53;

const struct MotorFactors {
  int straightSpeedFactor;
  float turnSpeedFastFactor;
  float turnSpeedSlowFactor;
} REGULAR_MOTOR_DRIVE = {
  200,
  1.0f,
  0.3f
};

#endif

