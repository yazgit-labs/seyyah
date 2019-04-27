#ifndef ODOMETER_H
#define ODOMETER_H

class Odometer {
public:
  Odometer();
  long oneTurnTick = 700;
  float oneTurnDistance = 19.5;
  long rightEncoderTick = 0;
  long leftEncoderTick = 0;
  float distanceDriven = 0;

  float getWay();

};


#endif
