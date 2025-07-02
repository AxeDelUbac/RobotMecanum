#ifndef CAPTEURHALL_H
#define CAPTEURHALL_H

#include <Arduino.h>
#include "encoderParameter.h"

class HallSensor {
  public:
    float getHallAngularSpeed();
    
    unsigned int hallPulses;

  private:
    
    float angularSpeed;
    
};

#endif