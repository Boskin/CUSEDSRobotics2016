#ifndef COMPASS_H
#define COMPASS_H

class Compass {
private:
  // I2C address of the device
  int address;
public:
  // Return an angle bearing of the direction the compass is facing
  float getAngleReading() const;
  
  Compass(int _address);
  ~Compass();
};

#endif
