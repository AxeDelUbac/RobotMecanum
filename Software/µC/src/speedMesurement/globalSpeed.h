#ifndef GLOBALSPEED_H
#define GLOBALSPEED_H

#include "rotaryEncoder.h"

class globalSpeed {
    public:
        float getGlobalSpeedKmh(int nbtour1, int nbtour2, int nbtour3, int nbtour4); // Retourne la vitesse en km/h
        float getGlobalSpeedRpm(int nbtour1, int nbtour2, int nbtour3, int nbtour4); // Retourne la vitesse en RPM

    private:
        float meanGlobalAngularSpeed; // Vitesse angulaire en RPM
        float meanGlobalLinearSpeed; // Vitesse linéaire en km/h

        rotaryEncoder rotaryEncoderFrontRight;
        rotaryEncoder rotaryEncoderFrontLeft;
        rotaryEncoder rotaryEncoderBackRight;
        rotaryEncoder rotaryEncoderBackLeft;

};

#endif
