#ifndef ENCODEUR_H
#define ENCODEUR_H

#include <Arduino.h>
#include "encoderParameter.h"

typedef struct {
  float fAngularSpeedInRpm;
  float fLinearSpeedInKmH;
  int iMotorDirection;
} rotaryEncoder_t;

float rotaryEncoder_getSpeedRpm(rotaryEncoder_t *enc, int ipulse, float fSpeedMesurementPeriod);
float rotaryEncoder_getSpeedKmH(rotaryEncoder_t *enc, int ipulse, float fSpeedMesurementPeriod);
int rotaryEncoder_getDirection(rotaryEncoder_t *enc, int iDirection);

#endif
