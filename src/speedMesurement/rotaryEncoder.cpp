#include "rotaryEncoder.h"

float sensor1LinearSpeed = 0;

float rotaryEncoder_getSpeedRpm(rotaryEncoder_t *enc, int ipulse, float fSpeedMesurementPeriod)
{
  enc->fAngularSpeedInRpm = (ipulse * 60000) / (pulsesPerRotation * MotorReductionRatio * fSpeedMesurementPeriod);
  return enc->fAngularSpeedInRpm;
}

float rotaryEncoder_getSpeedKmH(rotaryEncoder_t *enc, int ipulse, float fSpeedMesurementPeriod)
{
  int SpeedKmh = rotaryEncoder_getSpeedRpm(enc, ipulse, fSpeedMesurementPeriod);
  enc->fLinearSpeedInKmH = SpeedKmh*PI*wheelDiameter/60*3.6;

  return enc->fLinearSpeedInKmH;
}

int rotaryEncoder_getDirection(rotaryEncoder_t *enc, int iDirection)
{
  enc->iMotorDirection = iDirection;
}