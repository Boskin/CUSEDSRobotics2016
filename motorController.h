#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "constants.h"

// One motor controller controls 2 motors
class MotorController {
private:
  // Information about the GPIO used to control the ports
  MotorControllerPorts ports;
  
public:
  static void standby(bool enable);
  static void setupStandby();
  /* Drive the motor with a given power (0-255) and a given
     direction */
  void drive(int power, int dir);

  MotorController(MotorControllerPorts _ports);
  ~MotorController();
};

#endif

