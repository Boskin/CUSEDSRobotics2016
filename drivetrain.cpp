#include <Arduino.h>
#include "drivetrain.h"

void Drivetrain::standby(bool enable) {
  digitalWrite(GLOBAL_STANDBY, enable);
}

void Drivetrain::setupStandby() {
  pinMode(GLOBAL_STANDBY, OUTPUT);
}

Drivetrain::Drivetrain() {
  standby(HIGH);
}

Drivetrain::Drivetrain(MotorControllerPorts _frontLeftPorts, MotorControllerPorts _frontRightPorts,
  MotorControllerPorts _midLeftPorts, MotorControllerPorts _midRightPorts,
  MotorControllerPorts _rearLeftPorts, MotorControllerPorts _rearRightPorts) {
  frontLeft = MotorController(_frontLeftPorts);
  frontRight = MotorController(_frontRightPorts);
  midLeft = MotorController(_midLeftPorts);
  midRight = MotorController(_midRightPorts);
  rearLeft = MotorController(_rearLeftPorts);
  rearRight = MotorController(_rearRightPorts);
}

Drivetrain::~Drivetrain() {}

