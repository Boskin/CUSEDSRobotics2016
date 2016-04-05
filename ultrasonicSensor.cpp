#include <Arduino.h> // Includes built-in Arduino fields like HIGH, OUTPUT, etc.
#include "ultrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor(): ports({0, 0}) {}

UltrasonicSensor::UltrasonicSensor(UltrasonicPort _ports): ports(_ports) {
  // Double check later
  pinMode(ports.trig, OUTPUT);
  pinMode(ports.echo, INPUT);
}

UltrasonicSensor::~UltrasonicSensor() {}

float UltrasonicSensor::getDistance() const {
  // Time it takes for the signal to bounce back and be read
  long duration;
  // Distance, which is directly proportional to the duration (cm)
  long distance;
  
  // Send a soundwave pulse
  digitalWrite(ports.trig, LOW);
  delayMicroseconds(2);
  digitalWrite(ports.trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(ports.trig, LOW);

  duration = pulseIn(ports.echo, HIGH);

  distance = duration * DURATION_DISTANCE_FACTOR;

  return distance;
}

