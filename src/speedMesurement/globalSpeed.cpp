#include "globalSpeed.h"

float globalSpeed::getGlobalSpeedRpm(int iPulseFrontRight, int iPulseFrontLeft, int iPulseBackRight, int iPulseBackLeft)
{
  float speedFrontRight = 0;
  float speedFrontLeft = 0;
  float speedBackRight = 0;
  float speedBackLeft = 0;

  speedFrontRight = this->rotaryEncoderFrontRight.getSpeedRpm(iPulseFrontRight);
  speedFrontLeft = this->rotaryEncoderFrontLeft.getSpeedRpm(iPulseFrontLeft);
  speedBackRight = this->rotaryEncoderBackRight.getSpeedRpm(iPulseBackRight);
  speedBackLeft = this->rotaryEncoderBackLeft.getSpeedRpm(iPulseBackLeft);

  // Serial.print(">Vitesse FrontRight:");
  // Serial.println(speedFrontRight);
  // Serial.print(">Vitesse Frontleft:");
  // Serial.println(speedFrontLeft);
  // Serial.print(">Vitesse BackRight:");
  // Serial.println(speedBackRight);
  // Serial.print(">Vitesse Backleft:");
  // Serial.println(speedBackLeft);

  this->meanGlobalAngularSpeed = ((speedFrontRight + speedFrontLeft+ speedBackRight + speedBackLeft) / 4);
  
  return this->meanGlobalAngularSpeed;
}

float globalSpeed::getGlobalSpeedKmh(int iPulseFrontRight, int iPulseFrontLeft, int iPulseBackRight, int iPulseBackLeft)
{
  float speedRpm = 0;
  speedRpm = this->getGlobalSpeedRpm(iPulseFrontRight,iPulseFrontLeft,iPulseBackRight,iPulseBackLeft);
  
  this->meanGlobalLinearSpeed = speedRpm * PI * wheelDiameter/60*3.6;

  return this->meanGlobalLinearSpeed;
}