#include "movementController.h"

movementController::movementController()
    : 
    oleftFrontMotor(leftFrontMotorHigh, leftFrontMotorLow, leftFrontMotorPWM),
    orightFrontMotor(rightFrontMotorHigh, rightFrontMotorLow, rightFrontMotorPWM),
    oleftBackMotor(leftBackMotorHigh, leftBackMotorLow, leftBackMotorPWM),
    orightBackMotor(rightBackMotorHigh, rightBackMotorLow, rightBackMotorPWM)
    {}

void movementController::setDirection(int Xaxis, int Yaxis, int iSpeed) {
    if (Xaxis == 0 && Yaxis == 1) { // Avant
        this->orightFrontMotor.setMotorDirectionPWM(1, iSpeed);
        this->oleftFrontMotor.setMotorDirectionPWM(1, iSpeed);
        this->orightBackMotor.setMotorDirectionPWM(1,iSpeed);
        this->oleftBackMotor.setMotorDirectionPWM(1,iSpeed);
    } else if (Xaxis == 0 && Yaxis == -1) { // Arrière
        this->orightFrontMotor.setMotorDirectionPWM(0, iSpeed);
        this->oleftFrontMotor.setMotorDirectionPWM(0, iSpeed);
        this->orightBackMotor.setMotorDirectionPWM(0,iSpeed);
        this->oleftBackMotor.setMotorDirectionPWM(0, iSpeed);
    } else if (Xaxis == 1 && Yaxis == 0) { // Droite
        this->orightFrontMotor.setMotorDirectionPWM(1, iSpeed);
        this->oleftFrontMotor.setMotorDirectionPWM(0, iSpeed);
        this->orightBackMotor.setMotorDirectionPWM(0,iSpeed);
        this->oleftBackMotor.setMotorDirectionPWM(1, iSpeed);
    } else if (Xaxis == -1 && Yaxis == 0) { // Gauche
        this->orightFrontMotor.setMotorDirectionPWM(1, iSpeed);
        this->oleftFrontMotor.setMotorDirectionPWM(0, iSpeed);
        this->orightBackMotor.setMotorDirectionPWM(0,iSpeed);
        this->oleftBackMotor.setMotorDirectionPWM(1, iSpeed);
    }
}

void movementController::setRotation(int iSpeed){

    int Xaxis; // Exemple de valeur pour Xaxis
    int Yaxis; // Exemple de valeur pour Yaxis

    if (Xaxis == 1 && Yaxis == 1) { // Rotation droite
        this->orightFrontMotor.setMotorDirectionPWM(1, iSpeed);
        this->oleftFrontMotor.setMotorDirectionPWM(0, iSpeed);
        this->orightBackMotor.setMotorDirectionPWM(1,iSpeed);
        this->oleftBackMotor.setMotorDirectionPWM(0, iSpeed);
    } else if (Xaxis == -1 && Yaxis == -1) { // Rotation gauche
        this->orightFrontMotor.setMotorDirectionPWM(0, iSpeed);
        this->oleftFrontMotor.setMotorDirectionPWM(1, iSpeed);
        this->orightBackMotor.setMotorDirectionPWM(0,iSpeed);
        this->oleftBackMotor.setMotorDirectionPWM(1, iSpeed);
    }
}

void movementController::movementFront(void){
    this->oleftFrontMotor.setMotorDirectionPWM(1,this->fTabSpeedInPwm[0]);
    this->orightFrontMotor.setMotorDirectionPWM(1,this->fTabSpeedInPwm[1]);
    this->oleftBackMotor.setMotorDirectionPWM(1,this->fTabSpeedInPwm[2]);
    this->orightBackMotor.setMotorDirectionPWM(1,this->fTabSpeedInPwm[3]);
}

void movementController::movementBack(void){
    this->oleftFrontMotor.setMotorDirectionPWM(0,this->fTabSpeedInPwm[0]);
    this->orightFrontMotor.setMotorDirectionPWM(0,this->fTabSpeedInPwm[1]);
    this->oleftBackMotor.setMotorDirectionPWM(0,this->fTabSpeedInPwm[2]);
    this->orightBackMotor.setMotorDirectionPWM(0,this->fTabSpeedInPwm[3]);
}

void movementController::movementLeft(void){
    this->oleftFrontMotor.setMotorDirectionPWM(1,this->fTabSpeedInPwm[0]);
    this->orightFrontMotor.setMotorDirectionPWM(0,this->fTabSpeedInPwm[1]);
    this->oleftBackMotor.setMotorDirectionPWM(0,this->fTabSpeedInPwm[2]);
    this->orightBackMotor.setMotorDirectionPWM(1,this->fTabSpeedInPwm[3]);
}

void movementController::movementRight(void){
    this->oleftFrontMotor.setMotorDirectionPWM(0, this->fTabSpeedInPwm[0]);
    this->orightFrontMotor.setMotorDirectionPWM(1, this->fTabSpeedInPwm[1]);
    this->oleftBackMotor.setMotorDirectionPWM(1,this->fTabSpeedInPwm[2]);
    this->orightBackMotor.setMotorDirectionPWM(0, this->fTabSpeedInPwm[3]);
}


void movementController::movementFrontLeft(void){
    this->orightFrontMotor.setMotorDirectionPWM(1, this->fTabSpeedInPwm[1]);
    this->oleftBackMotor.setMotorDirectionPWM(1, this->fTabSpeedInPwm[2]);
}

void movementController::movementFrontRight(void){
    this->oleftFrontMotor.setMotorDirectionPWM(1, this->fTabSpeedInPwm[0]);
    this->orightBackMotor.setMotorDirectionPWM(1, this->fTabSpeedInPwm[3]);
}
  
void movementController::movementstop(){  
    this->orightBackMotor.stopMotor();
    this->oleftBackMotor.stopMotor();
    this->orightFrontMotor.stopMotor();
    this->oleftFrontMotor.stopMotor();
}

void movementController::setMotorSpeedInPWM(float fTabSpeed[4])
{
    for (int i = 0; i < 4; i++) {
        this->fTabSpeedInPwm[i] = fTabSpeed[i];
    }
}