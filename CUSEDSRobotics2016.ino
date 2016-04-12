#include <Wire.h>
#include "constants.h"
#include "compass.h"
#include "drivetrain.h"
#include "ultrasonicSensor.h"

Drivetrain drivetrain;

UltrasonicSensor leftUltrasonic;
UltrasonicSensor centerUltrasonic;
UltrasonicSensor rightUltrasonic;

Compass compass;

int currentRobotState;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
  Wire.begin();
  Drivetrain::setupStandby();
  
  drivetrain = Drivetrain(
    MOTOR_CONTROLLER_PORTS[0],
    MOTOR_CONTROLLER_PORTS[1],
    MOTOR_CONTROLLER_PORTS[2],
    MOTOR_CONTROLLER_PORTS[3],
    MOTOR_CONTROLLER_PORTS[4],
    MOTOR_CONTROLLER_PORTS[5]
  );

  leftUltrasonic = UltrasonicSensor(ULTRASONIC_PORTS[0]);
  centerUltrasonic = UltrasonicSensor(ULTRASONIC_PORTS[1]);
  rightUltrasonic = UltrasonicSensor(ULTRASONIC_PORTS[2]);

  compass = Compass(COMPASS_I2C_ADDRESS);
  compass.prepare();

  currentRobotState = DRIVE_TOWARD_GOAL;
  Drivetrain::standby(HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
