#ifndef MOTORGEARBOX_H
#define MOTORGEARBOX_H

#include <Arduino.h>

class motorGearBox {
    public:

        motorGearBox(int highPin, int lowPin, int pwmPin);

        void setMotorDirection(bool direction, int pourcentSpeed); // Stoppe le moteur
        void stopMotor(); // Définit la vitesse du moteur

    private:
        int highPin; // Pin de commande haute
        int lowPin; // Pin de commande basse
        int pwmPin; // Pin PWM pour la vitesse

};

#endif
