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

float getBearingRads() {
  return 0.0f; // Write this later
}

bool equalWithinTolerance(float a, float b, float tolerance) {
  float bLower = (1 - tolerance) * b;
  float bUpper = (1 + tolerance) * b;
  return a > bLower && a < bUpper;
}

int avoidObstacle(int dirState) {
  static float lastSensorReading;
  float currentSensorReading;
  int dirTurn;

  if(dirState == AVOID_OBSTACLE_LEFT) {
    currentSensorReading = rightUltrasonic.getDistance();
    dirTurn = DIR_LEFT;
  } else if(dirState == AVOID_OBSTACLE_RIGHT) {
    currentSensorReading = leftUltrasonic.getDistance();
    dirTurn = DIR_RIGHT;
  } else {
    return -1;
  }

  if(currentSensorReading - lastSensorReading < 0.0f || lastSensorReading == 0.0f) {
    drivetrain.turn(REGULAR_MOTOR_DRIVE.straightSpeedFactor, dirTurn);
    lastSensorReading = currentSensorReading;
    return dirState;
  } else {
    lastSensorReading = 0.0f;
    return dirTurn == DIR_LEFT ? PROCEED_LEFT : PROCEED_RIGHT;
  }
}

int proceed(int dirState) {
  
}

int driveTowardGoal() {
  float compassBearing = compass.getAngleReading();
  float beaconBearing = getBearingRads();

  drivetrain.driveStraight(REGULAR_MOTOR_DRIVE.straightSpeedFactor);

  if(centerUltrasonic.getDistance() <= OBJECT_TOO_CLOSE) {
    float leftReading = leftUltrasonic.getDistance();
    float rightReading = rightUltrasonic.getDistance();

    float dirToChoose = leftReading >= rightReading ? leftReading : rightReading;

    drivetrain.driveStraight(0);
    if(dirToChoose > OBJECT_TOO_CLOSE) {
      return leftReading >= rightReading ? AVOID_OBSTACLE_LEFT : AVOID_OBSTACLE_RIGHT;
    } else {
      return AVOID_OBSTACLE_BACK;
    }
  } else {
    return DRIVE_TOWARD_GOAL;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  switch(currentRobotState) {
  case DRIVE_TOWARD_GOAL:
    break;
  case AVOID_OBSTACLE_LEFT:
  case AVOID_OBSTACLE_RIGHT:
    break;
  default:
    Drivetrain::standby(LOW);
  }
}
