#include <Arduino.h>
#include "drivetrain.h"

void Drivetrain::standby(bool enable) {
  digitalWrite(GLOBAL_STANDBY, enable);
}

void Drivetrain::setupStandby() {
  pinMode(GLOBAL_STANDBY, OUTPUT);
}

void Drivetrain::driveStraight(int power) {
  frontRight.drive(power);
  midRight.drive(power);
  rearRight.drive(power);

  frontLeft.drive(-power);
  midLeft.drive(-power);
  rearLeft.drive(-power);
}

void Drivetrain::turn(int power, int dir) {
  int rightPower;
  int leftPower;
  
  if(dir == RIGHT) {
    rightPower = power * REGULAR_MOTOR_DRIVE.turnSpeedSlowFactor;
    leftPower = -power * REGULAR_MOTOR_DRIVE.turnSpeedFastFactor;
  } else if(dir == LEFT) {
    rightPower = power * REGULAR_MOTOR_DRIVE.turnSpeedFastFactor;
    leftPower = -power * REGULAR_MOTOR_DRIVE.turnSpeedSlowFactor;
  } else {
    rightPower = 0;
    leftPower = 0;
  }

  frontRight.drive(rightPower);
  midRight.drive(rightPower);
  rearRight.drive(rightPower);

  frontLeft.drive(leftPower);
  midLeft.drive(leftPower);
  rearLeft.drive(leftPower);
}

Drivetrain::Drivetrain() {
  standby(LOW);
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

Drivetrain::~Drivetrain() {
  standby(LOW);
}

