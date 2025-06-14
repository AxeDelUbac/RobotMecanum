#ifndef DIRECTIONCONTROLLER_H
#define DIRECTIONCONTROLLER_H

#include "motorGearBox.h"
#include "main.h"

class movementController {
    public:

        movementController();

        void setDirection(int Xaxis, int Yaxis, int iSpeed);
        void setRotation(int iSpeed);
        
        void movementFront(int iSpeed);
        void movementBack(int iSpeed);
        void movementRight(int iSpeed);
        void movementLeft(int iSpeed);

        void movementFrontLeft(int iSpeed);
        void movementFrontRight(int iSpeed);
        void movementBackLeft(int iSpeed);
        void movementBackRight(int iSpeed);

        void rotationRight(int iSpeed);
        void rotationLeft(int iSpeed);

        void movementstop();

    private:

        motorGearBox rightBackMotor;
        motorGearBox leftBackMotor;
        motorGearBox rightFrontMotor;
        motorGearBox leftFrontMotor;

};

#endif