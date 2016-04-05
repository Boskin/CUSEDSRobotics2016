#ifndef CONSTANTS_H
#define CONSTANTS_H

// These are all subject to change
const struct UltrasonicPort {
  int trig;
  int echo;
} ULTRASONIC_PORTS[] = {
  {22, 23}, // Trig and echo for sensor 0
  {24, 25} // Trig and echo for sensor 1
};

// Factor to multiply duration by to get distance
const float DURATION_DISTANCE_FACTOR = 1 / 58.2;

// Work in progress
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
  {2, 28, 29}, // PWM, IN1, IN2
  {3, 30, 31},
  {4, 32, 33},
  {5, 34, 35},
  {6, 36, 37},
  {7, 38, 39}
};

const int COUNTER_CLOCKWISE = 0;
const int CLOCKWISE = 1;

// All motors will share the same standby port
const int GLOBAL_STANDBY = 26;

const struct MotorFactors {
  float straightSpeedFactor;
  float turnSpeedFastFactor;
  float turnSpeedSlowFactor;
} REGULAR_MOTOR_DRIVE = {
  0.5f,
  0.6f,
  0.4f
};

#endif
