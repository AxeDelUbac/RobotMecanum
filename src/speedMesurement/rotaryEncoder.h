#ifndef ENCODEUR_H
#define ENCODEUR_H

#include <Arduino.h>
#include "encoderParameter.h"

#include "hallSensor.h"

class rotaryEncoder {
    public:

        float getSpeedRpm(int ipulse, float fSpeedMesurementPeriod);
        float getSpeedKmH(int ipulse, float fSpeedMesurementPeriod);
        int getDirection(int iDirection);

    private:
        float meanAngularSpeed;
        float meanLinearSpeed;
        int iMotorDirection;

        HallSensor sensor1;
        HallSensor sensor2; 
};

#endif
