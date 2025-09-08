#ifndef GLOBALSPEED_H
#define GLOBALSPEED_H

#include <Arduino.h>
#include "encoderParameter.h"

#include "rotaryEncoder.h"

class globalSpeed {
    public:
    
        float getGlobalSpeedKmh(int iPulseFrontRight, int iPulseFrontLeft, int iPulseBackRight, int iPulseBackLeft, float fSpeedMesurementPeriod);
        float getGlobalSpeedRpm(int iPulseFrontRight, int iPulseFrontLeft, int iPulseBackRight, int iPulseBackLeft, float fSpeedMesurementPeriod);
        void getMotorsRpm(int iPulseFrontRight, int iPulseFrontLeft, int iPulseBackRight, int iPulseBackLeft, float fSpeedMesurementPeriod, float fTabMotorSpeed[4]);
        void serialDebug(void);

    private:

        rotaryEncoder rotaryEncoderFrontRight;
        rotaryEncoder rotaryEncoderFrontLeft;
        rotaryEncoder rotaryEncoderBackRight;
        rotaryEncoder rotaryEncoderBackLeft;

        float fspeedFrontRightRPM = 0;
        float fspeedFrontLeftRPM = 0;
        float fspeedBackRightRPM = 0;
        float fspeedBackLeftRPM = 0;

        float meanGlobalAngularSpeed; // Vitesse angulaire en RPM
        float meanGlobalLinearSpeed; // Vitesse linéaire en km/h

        float ftabMotorSpeedRPM[4] = {0, 0, 0, 0};

        



};

#endif
