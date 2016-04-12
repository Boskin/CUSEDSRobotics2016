#include <Arduino.h>
#include <Wire.h>
#include "compass.h"

Compass::Compass(): address(-1) {}

Compass::Compass(int _address): address(_address) {
  referenceAngle = 0.0f;
  referenceAngle = getAngleReading();
}

Compass::~Compass() {}

float Compass::getAngleReading() const {
  int x;
  int y;
  int z;
  float heading;

  Wire.beginTransmission(address);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.requestFrom(address, 6);

  if(6 <= Wire.available()) {
    x = Wire.read() << 8;
    x |= Wire.read();
    z = Wire.read() << 8;
    z |= Wire.read();
    y = Wire.read() << 8;
    y |= Wire.read();
  }

  heading = 180 * atan2(y, x) / PI;
  if(heading < 0) {
    heading += 360.0f;
  }

  return heading - referenceAngle;
}

void Compass::prepare() {
  Wire.beginTransmission(address);
  Serial.println(Wire.write(0x02));
  Serial.println(Wire.write(0x00));
  Wire.endTransmission();
  Serial.println("Setup complete");
}

