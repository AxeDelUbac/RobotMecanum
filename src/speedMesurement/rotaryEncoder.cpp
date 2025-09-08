#include "rotaryEncoder.h"

float sensor1AngularSpeed = 0;
float sensor1LinearSpeed = 0;

float rotaryEncoder::getSpeedRpm(int ipulse, float fSpeedMesurementPeriod)
{
  sensor1.hallPulses = ipulse;
  sensor1AngularSpeed = sensor1.getHallAngularSpeed()* (1000 /fSpeedMesurementPeriod);// Convert to RPM based on measurement period


  return sensor1AngularSpeed;
}

float rotaryEncoder::getSpeedKmH(int ipulse, float fSpeedMesurementPeriod)
{
  int SpeedKmh = this->getSpeedRpm(ipulse, fSpeedMesurementPeriod);
  sensor1LinearSpeed = SpeedKmh*PI*wheelDiameter/60*3.6;

  return sensor1LinearSpeed;
}

int rotaryEncoder::getDirection(int iDirection)
{
  iMotorDirection = iDirection;
}