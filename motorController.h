#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "constants.h"

// One motor controller controls 2 motors
class MotorController {
private:
  // Information about the GPIO used to control the ports
  MotorControllerPorts ports;
  
public:
  /* Drive the motor with the given power, negative value means 
     clockwise */
  void drive(int power);

  MotorController();
  MotorController(MotorControllerPorts _ports);
  ~MotorController();
};

#endif

