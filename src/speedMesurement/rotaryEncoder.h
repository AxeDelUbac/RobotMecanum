#ifndef ENCODEUR_H
#define ENCODEUR_H

#include "hallSensor.h"

class rotaryEncoder {
    public:
        float getMeanSpeedKmh(int pulse1, int pulse2); // Retourne la vitesse en km/h
        float getMeanSpeedRpm(int pulse1, int pulse2); // Retourne la vitesse en RPM

        float getSpeedRpm(); // Retourne la vitesse en km/h
        int getDirection();

    private:
        float meanAngularSpeed; // Vitesse angulaire en RPM
        float meanLinearSpeed; // Vitesse linéaire en km/h

        HallSensor sensor1;
        HallSensor sensor2; 
};

#endif
