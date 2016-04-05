#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "constants.h"
#include "motorController.h"

class Drivetrain {
private:
  MotorController frontLeft;
  MotorController frontRight;
  MotorController midLeft;
  MotorController midRight;
  MotorController rearLeft;
  MotorController rearRight;
  
public:
  static void standby(bool enable);
  static void setupStandby();

  Drivetrain();
  Drivetrain(MotorControllerPorts _frontLeftPorts, MotorControllerPorts _frontRightPorts,
    MotorControllerPorts _midLeftPorts, MotorControllerPorts _midRightPorts,
    MotorControllerPorts _rearLeftPorts, MotorControllerPorts _rearRightPorts);
  ~Drivetrain();
};

#endif

