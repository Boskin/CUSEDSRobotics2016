#include <Arduino.h>
#include "motorController.h"

MotorController::MotorController(): ports({0, 0, 0}) {}

MotorController::MotorController(MotorControllerPorts _ports): ports(_ports) {
  // Setup pins that control motor a
  pinMode(ports.pwm, OUTPUT);
  pinMode(ports.in1, OUTPUT);
  pinMode(ports.in2, OUTPUT);
}

MotorController::~MotorController() {}

void MotorController::drive(int power) {
  // Control signals sent to either motor of the mode control pins
  bool dirControl1;
  bool dirControl2;

  if(power > 0) {
    // Let the motor rotate counter-clockwise
    dirControl1 = HIGH;
    dirControl2 = LOW;
  } else if(power < 0) {
    // Let the motor rotate clockwise
    dirControl1 = LOW;
    dirControl2 = HIGH;
    power = -power;
  } else {
    // If an invalid dir value was given, just stop the motor
    dirControl1 = LOW;
    dirControl2 = LOW;
  }

  digitalWrite(ports.in1, dirControl1);
  digitalWrite(ports.in2, dirControl2);
  analogWrite(ports.pwm, power);
}

