#include "GlobalSpeed.h"

GlobalSpeed_t tGlobalSpeed;

extern volatile int iNombreTour[4];
extern volatile int iDirection[4];

float GlobalSpeed_getMeanSpeedInRPM(float *fSpeedEncodeRPM,float fSpeedMesurementPeriodMs)
{

    tGlobalSpeed.fWheelSpeed[0] = rotaryEncoder_getSpeedRpm(&tGlobalSpeed.rotaryEncoderFrontLeft, iNombreTour[1], fSpeedMesurementPeriodMs);
    tGlobalSpeed.fWheelSpeed[1] = rotaryEncoder_getSpeedRpm(&tGlobalSpeed.rotaryEncoderFrontRight, iNombreTour[2], fSpeedMesurementPeriodMs);
    tGlobalSpeed.fWheelSpeed[2] = rotaryEncoder_getSpeedRpm(&tGlobalSpeed.rotaryEncoderRearLeft, iNombreTour[3], fSpeedMesurementPeriodMs);
    tGlobalSpeed.fWheelSpeed[3] = rotaryEncoder_getSpeedRpm(&tGlobalSpeed.rotaryEncoderRearRight, iNombreTour[0], fSpeedMesurementPeriodMs);

    for (int i = 0; i < 4; i++)
    {
        fSpeedEncodeRPM[i] = tGlobalSpeed.fWheelSpeed[i];
        iNombreTour[i] = 0;
    }

    tGlobalSpeed.fMeanRpm = (tGlobalSpeed.fWheelSpeed[0] + tGlobalSpeed.fWheelSpeed[1] + tGlobalSpeed.fWheelSpeed[2] + tGlobalSpeed.fWheelSpeed[3]) / 4;

  return tGlobalSpeed.fMeanRpm;
}

void GlobalSpeed_debug(void)
{
    Serial.print(">FrontLeft RPM :");
    Serial.println(tGlobalSpeed.fWheelSpeed[0]);
    Serial.print(">FrontRight RPM :");
    Serial.println(tGlobalSpeed.fWheelSpeed[1]);
    Serial.print(">BackLeft RPM :");
    Serial.println(tGlobalSpeed.fWheelSpeed[2]);
    Serial.print(">BackRight RPM :");
    Serial.println(tGlobalSpeed.fWheelSpeed[3]);

    Serial.print(">Mean Speed (RPM) :");
    Serial.println(tGlobalSpeed.fMeanRpm);

}