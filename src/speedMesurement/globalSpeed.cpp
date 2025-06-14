#include "globalSpeed.h"
#include "encoderParameter.h"
#include <Arduino.h>

float globalSpeed::getGlobalSpeedRpm(int nbtour1, int nbtour2, int nbtour3, int nbtour4){
  float speedBackRight = 0;
  float speedBackLeft = 0;

  speedBackRight = this->rotaryEncoderBackRight.getMeanSpeedKmh(nbtour1, nbtour2);
  speedBackLeft = this->rotaryEncoderBackLeft.getMeanSpeedKmh(nbtour3, nbtour4);

  Serial.print("Vitesse en RPM1 = ");
  Serial.println(speedBackRight);
  Serial.print("Vitesse en RPM2 = ");
  Serial.println(speedBackLeft);

  this->meanGlobalAngularSpeed = ((speedBackRight + speedBackLeft) / 2);        
  
    return this->meanGlobalAngularSpeed;
  }

  float globalSpeed::getGlobalSpeedKmh(int nbtour1, int nbtour2, int nbtour3, int nbtour4){
    float speedRpm = 0;
    speedRpm = this->getGlobalSpeedRpm(nbtour1,nbtour2,nbtour3,nbtour4);
   
    this->meanGlobalLinearSpeed = (speedRpm * PI * wheelDiameter) / (60 * 3.6);

      return this->meanGlobalLinearSpeed;
    }