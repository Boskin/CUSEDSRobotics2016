#include <Arduino.h>
#include <Wire.h>
#include "compass.h"

Compass::Compass(int _address): address(_address) {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(address);
  Serial.println(Wire.write(0x02));
  Serial.println(Wire.write(0x00));
  Wire.endTransmission();
  Serial.println("Setup complete");
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

  heading = atan2(y, x);
  if(heading < 0) {
    heading += 2 * PI;
  }

  return heading;
}

