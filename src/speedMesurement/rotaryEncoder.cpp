#include "rotaryEncoder.h"

float sensor1AngularSpeed = 0;
float sensor1LinearSpeed = 0;

float rotaryEncoder::getSpeedRpm(int ipulse)
{
  sensor1.hallPulses = ipulse;
  sensor1AngularSpeed = sensor1.getHallAngularSpeed();

  return sensor1AngularSpeed;
}

float rotaryEncoder::getSpeedKmH(int ipulse)
{
  int SpeedKmh = this->getSpeedRpm(ipulse);
  sensor1LinearSpeed = SpeedKmh*PI*wheelDiameter/60*3.6;

  return sensor1LinearSpeed;
}

int rotaryEncoder::getDirection(int iDirection)
{
  iMotorDirection = iDirection;
}