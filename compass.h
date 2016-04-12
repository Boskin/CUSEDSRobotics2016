#ifndef COMPASS_H
#define COMPASS_H

class Compass {
private:
  // I2C address of the device
  int address;

  float referenceAngle;
public:
  // Return an angle bearing of the direction the compass is facing
  float getAngleReading() const;

  float getReferenceAngle() const;

  void prepare();

  Compass();
  Compass(int _address);
  ~Compass();
};

#endif

