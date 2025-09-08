#include "globalSpeed.h"

float globalSpeed::getGlobalSpeedRpm(int iPulseFrontRight, int iPulseFrontLeft, int iPulseBackRight, int iPulseBackLeft, float fSpeedMesurementPeriod)
{
  fspeedFrontRightRPM = 0;
  fspeedFrontLeftRPM = 0;
  fspeedBackRightRPM = 0;
  fspeedBackLeftRPM = 0;

  fspeedFrontRightRPM = this->rotaryEncoderBackLeft.getSpeedRpm(iPulseFrontRight,fSpeedMesurementPeriod);
  fspeedFrontLeftRPM = this->rotaryEncoderFrontLeft.getSpeedRpm(iPulseFrontLeft,fSpeedMesurementPeriod);
  fspeedBackRightRPM = this->rotaryEncoderBackRight.getSpeedRpm(iPulseBackRight,fSpeedMesurementPeriod);
  fspeedBackLeftRPM = this->rotaryEncoderFrontRight.getSpeedRpm(iPulseBackLeft,fSpeedMesurementPeriod);

  this->meanGlobalAngularSpeed = ((fspeedFrontRightRPM + fspeedFrontLeftRPM+ fspeedBackRightRPM + fspeedBackLeftRPM) / 4);
  
  return this->meanGlobalAngularSpeed;
}

float globalSpeed::getGlobalSpeedKmh(int iPulseFrontRight, int iPulseFrontLeft, int iPulseBackRight, int iPulseBackLeft,float fSpeedMesurementPeriod)
{
  float speedRpm = 0;
  speedRpm = this->getGlobalSpeedRpm(iPulseFrontRight,iPulseFrontLeft,iPulseBackRight,iPulseBackLeft, fSpeedMesurementPeriod);
  
  this->meanGlobalLinearSpeed = speedRpm * PI * wheelDiameter/60*3.6;

  return this->meanGlobalLinearSpeed;
}


void globalSpeed::serialDebug(void) {
    Serial.print(">FrontLeft RPM :");
    Serial.println(fspeedFrontLeftRPM);
    Serial.print(">FrontRight RPM :");
    Serial.println(fspeedFrontRightRPM);
    Serial.print(">BackLeft RPM :");
    Serial.println(fspeedBackLeftRPM);
    Serial.print(">BackRight RPM :");
    Serial.println(fspeedBackRightRPM);

    Serial.print(">Mean Speed (RPM) :");
    Serial.println(meanGlobalAngularSpeed);
    Serial.print(">Mean Speep(km/h) :");
    Serial.println(meanGlobalLinearSpeed);
}