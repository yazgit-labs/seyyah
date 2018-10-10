#include "Odometer.h"

Odometer::Odometer(){};

float Odometer::getWay(){
  float result;
  result = float(rightEncoderTick + leftEncoderTick)/2.0;
  result = result / float(oneTurnTick);
  result = result * oneTurnDistance;

  return result;

}
