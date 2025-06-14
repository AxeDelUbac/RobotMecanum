#include "hallSensor.h"
#include "encoderParameter.h"
#include <Arduino.h>

float HallSensor::getHallAngularSpeed() {
  this->angularSpeed = (this->hallPulses*60)/ pulsesPerRotation / MotorReductionRatio;
  return this->angularSpeed;
}

int HallSensor::getPulses() {
  return this->hallPulses;
}