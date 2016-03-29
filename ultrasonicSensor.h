#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "constants.h"

class UltrasonicSensor {
private:
  UltrasonicPort ports;
  
public:
  float getDistance() const;
  
  UltrasonicSensor(UltrasonicPort _ports);
  ~UltrasonicSensor();
};

#endif

