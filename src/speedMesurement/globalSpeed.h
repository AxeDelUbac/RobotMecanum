#ifndef GLOBALSPEED_H
#define GLOBALSPEED_H

#include <Arduino.h>
#include "encoderParameter.h"

#include "rotaryEncoder.h"

class globalSpeed {
    public:
    
        float getGlobalSpeedKmh(int iPulseFrontRight, int iPulseFrontLeft, int iPulseBackRight, int iPulseBackLeft);
        float getGlobalSpeedRpm(int iPulseFrontRight, int iPulseFrontLeft, int iPulseBackRight, int iPulseBackLeft);

    private:

        float meanGlobalAngularSpeed; // Vitesse angulaire en RPM
        float meanGlobalLinearSpeed; // Vitesse linéaire en km/h

        rotaryEncoder rotaryEncoderFrontRight;
        rotaryEncoder rotaryEncoderFrontLeft;
        rotaryEncoder rotaryEncoderBackRight;
        rotaryEncoder rotaryEncoderBackLeft;

};

#endif
