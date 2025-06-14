#include "rotaryEncoder.h"
#include "encoderParameter.h"
#include <Arduino.h>

  float rotaryEncoder::getMeanSpeedRpm(int pulse1, int pulse2){
    float sensor1AngularSpeed = 0;
    float sensor2AngularSpeed = 0;

        this->sensor1.hallPulses = pulse1;
        this->sensor2.hallPulses = pulse2;

        sensor1AngularSpeed = this->sensor1.getHallAngularSpeed();
        sensor2AngularSpeed = this->sensor2.getHallAngularSpeed();

        this->meanAngularSpeed= ((sensor1AngularSpeed + sensor2AngularSpeed) / 2);

    return this->meanAngularSpeed;
  }

  float rotaryEncoder::getMeanSpeedKmh(int pulse1, int pulse2){

    int MeanSpeedRpm = this->getMeanSpeedRpm(pulse1, pulse2);
    this->meanLinearSpeed = MeanSpeedRpm*PI*wheelDiameter/60*3.6;

    return this->meanLinearSpeed;
  }

  int rotaryEncoder::getDirection(){
  }