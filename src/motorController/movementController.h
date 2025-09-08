#ifndef DIRECTIONCONTROLLER_H
#define DIRECTIONCONTROLLER_H

#include <Arduino.h>
#include "GPIO.h"

#include "motorGearBox.h"

class movementController {
    public:

        movementController();

        void setDirection(int Xaxis, int YaxisZ,  int iSpeed); // Xaxis et Yaxis sont les valeurs de l'accéléromètre, iTabSpeedInPwm est le tableau des vitesses PWM pour chaque moteur
        void setRotation(int iSpeed);
        
        void movementFront(void);
        void movementBack(void);
        void movementRight(void);
        void movementLeft(void);

        void movementFrontLeft(void);
        void movementFrontRight(void);

        void movementstop(void);

        void setMotorSpeedInPWM(float fTabSpeed[4]);

    private:

        motorGearBox orightBackMotor;
        motorGearBox oleftBackMotor;
        motorGearBox orightFrontMotor;
        motorGearBox oleftFrontMotor;

        float fTabSpeedInPwm[4] = {0, 0, 0, 0};

};

#endif