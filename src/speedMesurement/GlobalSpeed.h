#ifndef GLOBALSPEED_H
#define GLOBALSPEED_H

#include <Arduino.h>
#include "encoderParameter.h"
#include "rotaryEncoder.h"

typedef struct {
    rotaryEncoder_t rotaryEncoderFrontLeft;
    rotaryEncoder_t rotaryEncoderFrontRight;
    rotaryEncoder_t rotaryEncoderRearLeft;
    rotaryEncoder_t rotaryEncoderRearRight;       // 0: FrontLeft, 1: FrontRight, 2: RearLeft, 3: RearRight
    float fWheelSpeed[4];               // vitesses par roue (RPM)
    float fMeanRpm;                  // moyenne RPM
} GlobalSpeed_t;

float GlobalSpeed_getMeanSpeedInRPM(float *fSpeedEncodeRPM,float fSpeedMesurementPeriodMs);
void GlobalSpeed_debug(void);

#endif