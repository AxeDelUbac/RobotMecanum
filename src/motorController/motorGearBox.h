#ifndef MOTORGEARBOX_H
#define MOTORGEARBOX_H

#include <Arduino.h>

class motorGearBox {
    public:

        motorGearBox(int highPin, int lowPin, int pwmPin);

        void setMotorDirectionPWM(bool iDirection, int iSpeedInPWM);
        void setMotorDirectionPercent(bool iDirection, int iSpeedInPercent);

        void stopMotor(); // Définit la vitesse du moteur

    private:
        int highPin; // Pin de commande haute
        int lowPin; // Pin de commande basse
        int pwmPin; // Pin PWM pour la vitesse

};

#endif
