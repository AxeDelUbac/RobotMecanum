#include "hallSensor.h"

float HallSensor::getHallAngularSpeed() {
  this->angularSpeed = (this->hallPulses*60)/ pulsesPerRotation / MotorReductionRatio;
  return this->angularSpeed;
}