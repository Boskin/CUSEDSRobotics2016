#ifndef ROBOT_STATE_MACHINE
#define ROBOT_STATE_MACHINE

#include "drivetrain.h"
#include "compass.h"
#include "ultrasonicSensor.h"

class RobotStateMachine {
private:
  Drivetrain drivetrain;
  Compass compass;
  UltrasonicSensor rightUltrasonic;
  UltrasonicSensor centerUltrasonic;
  UltrasonicSensor leftUltrasonic;
  int currentState;

public:
  void stateControl();

  int avoidObstacle();
  int avoidObstacleTrapped();
  int proceed();
  int driveTowardGoal();
  int alignWithBeacon();

  float getAngleToRotateTo();

  int getState();
  void setState(int _currentState);

  RobotStateMachine();
  RobotStateMachine(
    Drivetrain _drivetrain,
    Compass _compass,
    UltrasonicSensor _rightUltrasonic,
    UltrasonicSensor _centerUltrasonic,
    UltrasonicSensor _leftUltrasonic
  );
  ~RobotStateMachine();
};

#endif

