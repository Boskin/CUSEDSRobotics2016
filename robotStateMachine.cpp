#include <Arduino.h>
#include "robotStateMachine.h"
#include "constants.h"

// This needs to be properly implemented by talking to the XBEE...
float getBeaconBearing() {
  int i = 0;
  delay(2);
  char bufferRead[256];
  while(Serial1.available()){
    bufferRead[i] = Serial1.read();
    i++;
  }
  //if(i > 0) {
    bufferRead[i - 1] = '\0';
    String number = bufferRead;
    return (float)number.toInt();
  /*} else {
    return 0;
  } */
}

bool equalWithinTolerance(float a, float b, float tolerance) {
  float bLower = b - tolerance;
  float bUpper = b + tolerance;

  return a >= bLower && a <= bUpper;
}

RobotStateMachine::RobotStateMachine() {}

RobotStateMachine::RobotStateMachine(
  Drivetrain _drivetrain,
  Compass _compass,
  UltrasonicSensor _rightUltrasonic,
  UltrasonicSensor _centerUltrasonic,
  UltrasonicSensor _leftUltrasonic
):
  drivetrain(_drivetrain),
  compass(_compass),
  rightUltrasonic(_rightUltrasonic),
  centerUltrasonic(_centerUltrasonic),
  leftUltrasonic(_leftUltrasonic) {
  currentState = DRIVE_TOWARD_GOAL;
}

RobotStateMachine::~RobotStateMachine() {}

void RobotStateMachine::stateControl() {
  switch(currentState) {
  case DRIVE_TOWARD_GOAL:
    currentState = driveTowardGoal();
    Serial.println("Drive toward goal");
    break;
  
  case AVOID_OBSTACLE_LEFT:
  case AVOID_OBSTACLE_RIGHT:
    currentState = avoidObstacle();
    Serial.println("Avoid obstacle");
    break;

  case AVOID_OBSTACLE_BACK:
    currentState = avoidObstacleTrapped();
    Serial.println("Avoid obstacle behind");
    break;

  case PROCEED_LEFT:
  case PROCEED_RIGHT:
    currentState = proceed();
    Serial.println("Proceed");
    break;

  case ALIGN_WITH_GOAL:
    currentState = alignWithBeacon();
    Serial.println("Align");
    break;

  default:
    Drivetrain::standby(LOW);
  }
}

int RobotStateMachine::avoidObstacle() {
  static float lastSensorReading;
  float currentSensorReading;
  int dirTurn;

  if(currentState == AVOID_OBSTACLE_LEFT) {
    currentSensorReading = rightUltrasonic.getDistance();
    dirTurn = DIR_LEFT;
  } else if(currentState == AVOID_OBSTACLE_RIGHT) {
    currentSensorReading = leftUltrasonic.getDistance();
    dirTurn = DIR_RIGHT;
  } else {
    return -1;
  }

  if(currentSensorReading - lastSensorReading < 0.0f || lastSensorReading == 0.0f ||
     centerUltrasonic.getDistance() <= OBJECT_TOO_CLOSE) {
    drivetrain.turn(REGULAR_MOTOR_DRIVE.straightSpeedFactor, dirTurn);
    lastSensorReading = currentSensorReading;
    return currentState;
  } else {
    lastSensorReading = 0.0f;
    return dirTurn == DIR_LEFT ? PROCEED_LEFT : PROCEED_RIGHT;
  }
}

int RobotStateMachine::avoidObstacleTrapped() {
  float leftReading = leftUltrasonic.getDistance();
  float rightReading = rightUltrasonic.getDistance();

  if(leftReading <= OBJECT_TOO_CLOSE || rightReading <= OBJECT_TOO_CLOSE) {
    drivetrain.driveStraight(-REGULAR_MOTOR_DRIVE.straightSpeedFactor);
    return currentState;
  } else {
    drivetrain.driveStraight(0);
    return ALIGN_WITH_GOAL;
  }
}

int RobotStateMachine::proceed() {
  float sensorReading;

  if(centerUltrasonic.getDistance() <= OBJECT_TOO_CLOSE) {
    drivetrain.driveStraight(0);
    if(currentState == PROCEED_LEFT) {
      return AVOID_OBSTACLE_LEFT;
    } else if(currentState == PROCEED_RIGHT) {
      return AVOID_OBSTACLE_RIGHT;
    } else {
      return -1;
    }
  }
  
  if(currentState == PROCEED_LEFT) {
    sensorReading = rightUltrasonic.getDistance();
  } else if(currentState == PROCEED_RIGHT) {
    sensorReading = leftUltrasonic.getDistance();
  } else {
    return -1;
  }

  if(sensorReading <= OBJECT_TOO_CLOSE) {
    drivetrain.driveStraight(REGULAR_MOTOR_DRIVE.straightSpeedFactor);
    return currentState;
  } else {
    drivetrain.driveStraight(0);
    return ALIGN_WITH_GOAL;
  }
}

int RobotStateMachine::driveTowardGoal() {
  float rotate = getAngleToRotateTo();

  /* if(!equalWithinTolerance(rotate, 0, ANGLE_EQUALITY_TOLERANCE)) {
    drivetrain.driveStraight(0);
    return ALIGN_WITH_GOAL;
  } */

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

int RobotStateMachine::alignWithBeacon() {
  float leftSensorReading = leftUltrasonic.getDistance();
  float centerSensorReading = centerUltrasonic.getDistance();
  float rightSensorReading = rightUltrasonic.getDistance();

  float rotate = getAngleToRotateTo();

  if(centerSensorReading <= OBJECT_TOO_CLOSE) {
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
    if(equalWithinTolerance(rotate, 0, ANGLE_EQUALITY_TOLERANCE)) {
      drivetrain.driveStraight(0);
      return DRIVE_TOWARD_GOAL;
    } else if(rotate > 180.0f) {
      // Turn left
      if(leftSensorReading > OBJECT_TOO_CLOSE) {
        drivetrain.turn(REGULAR_MOTOR_DRIVE.straightSpeedFactor, DIR_LEFT);
      } else {
        drivetrain.turn(REGULAR_MOTOR_DRIVE.straightSpeedFactor, DIR_RIGHT);
      }
    } else {
      // Turn right
      if(rightSensorReading > OBJECT_TOO_CLOSE) {
        drivetrain.turn(REGULAR_MOTOR_DRIVE.straightSpeedFactor, DIR_RIGHT);
      } else {
        drivetrain.turn(REGULAR_MOTOR_DRIVE.straightSpeedFactor, DIR_LEFT);
      }
    }
  }

  return currentState;
}

float RobotStateMachine::getAngleToRotateTo() {
  float beaconReading = getBeaconBearing();
  float compassReading = compass.getAngleReading();

  if(beaconReading > 180) {
    return compassReading - beaconReading + 180;
  } else {
    return compassReading - beaconReading - 180;
  }
}

int RobotStateMachine::getState() {
  return currentState;
}

void RobotStateMachine::setState(int _currentState) {
  currentState = _currentState;
}

