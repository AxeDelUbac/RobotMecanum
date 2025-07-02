#include "movementController.h"

movementController::movementController()
    : 
    oleftFrontMotor(leftFrontMotorHigh, leftFrontMotorLow, leftFrontMotorPWM),
    orightFrontMotor(rightFrontMotorHigh, rightFrontMotorLow, rightFrontMotorPWM),
    oleftBackMotor(leftBackMotorHigh, leftBackMotorLow, leftBackMotorPWM),
    orightBackMotor(rightBackMotorHigh, rightBackMotorLow, rightBackMotorPWM)
    {}

void movementController::setDirection(int Xaxis, int Yaxis, int iSpeed){
    if (Xaxis == 0 && Yaxis == 1) { // Avant
        this->orightFrontMotor.setMotorDirection(1, iSpeed);
        this->oleftFrontMotor.setMotorDirection(1, iSpeed);
        this->orightBackMotor.setMotorDirection(1,iSpeed);
        this->oleftBackMotor.setMotorDirection(1,iSpeed);
    } else if (Xaxis == 0 && Yaxis == -1) { // Arrière
        this->orightFrontMotor.setMotorDirection(0, iSpeed);
        this->oleftFrontMotor.setMotorDirection(0, iSpeed);
        this->orightBackMotor.setMotorDirection(0,iSpeed);
        this->oleftBackMotor.setMotorDirection(0, iSpeed);
    } else if (Xaxis == 1 && Yaxis == 0) { // Droite
        this->orightFrontMotor.setMotorDirection(1, iSpeed);
        this->oleftFrontMotor.setMotorDirection(0, iSpeed);
        this->orightBackMotor.setMotorDirection(0,iSpeed);
        this->oleftBackMotor.setMotorDirection(1, iSpeed);
    } else if (Xaxis == -1 && Yaxis == 0) { // Gauche
        this->orightFrontMotor.setMotorDirection(1, iSpeed);
        this->oleftFrontMotor.setMotorDirection(0, iSpeed);
        this->orightBackMotor.setMotorDirection(0,iSpeed);
        this->oleftBackMotor.setMotorDirection(1, iSpeed);
    }
}

void movementController::setRotation(int iSpeed){

    int Xaxis; // Exemple de valeur pour Xaxis
    int Yaxis; // Exemple de valeur pour Yaxis

    if (Xaxis == 1 && Yaxis == 1) { // Rotation droite
        this->orightFrontMotor.setMotorDirection(1, iSpeed);
        this->oleftFrontMotor.setMotorDirection(0, iSpeed);
        this->orightBackMotor.setMotorDirection(1,iSpeed);
        this->oleftBackMotor.setMotorDirection(0, iSpeed);
    } else if (Xaxis == -1 && Yaxis == -1) { // Rotation gauche
        this->orightFrontMotor.setMotorDirection(0, iSpeed);
        this->oleftFrontMotor.setMotorDirection(1, iSpeed);
        this->orightBackMotor.setMotorDirection(0,iSpeed);
        this->oleftBackMotor.setMotorDirection(1, iSpeed);
    }
}

void movementController::movementFront(int iSpeed){
    this->orightFrontMotor.setMotorDirection(1, iSpeed);
    this->oleftFrontMotor.setMotorDirection(1, iSpeed);
    this->orightBackMotor.setMotorDirection(1,iSpeed);
    this->oleftBackMotor.setMotorDirection(1, iSpeed);
}

void movementController::movementBack(int iSpeed){
    this->orightFrontMotor.setMotorDirection(0, iSpeed);
    this->oleftFrontMotor.setMotorDirection(0, iSpeed);
    this->orightBackMotor.setMotorDirection(0,iSpeed);
    this->oleftBackMotor.setMotorDirection(0, iSpeed);
}

void movementController::movementLeft(int iSpeed){
    this->orightFrontMotor.setMotorDirection(1, iSpeed);
    this->oleftFrontMotor.setMotorDirection(0, iSpeed);
    this->orightBackMotor.setMotorDirection(0,iSpeed);
    this->oleftBackMotor.setMotorDirection(1, iSpeed);
}

void movementController::movementRight(int iSpeed){
    this->orightFrontMotor.setMotorDirection(0, iSpeed);
    this->oleftFrontMotor.setMotorDirection(1, iSpeed);
    this->orightBackMotor.setMotorDirection(1,iSpeed);
    this->oleftBackMotor.setMotorDirection(0, iSpeed);
}


void movementController::movementFrontLeft(int iSpeed){
    this->orightFrontMotor.setMotorDirection(1, iSpeed);
    this->oleftBackMotor.setMotorDirection(1, iSpeed);
}

void movementController::movementFrontRight(int iSpeed){
    this->oleftFrontMotor.setMotorDirection(1, iSpeed);
    this->orightBackMotor.setMotorDirection(1, iSpeed);
}


void movementController::movementBackLeft(int iSpeed){
    this->oleftFrontMotor.setMotorDirection(0, iSpeed);
    this->orightBackMotor.setMotorDirection(0, iSpeed);
}

void movementController::movementBackRight(int iSpeed){
    this->oleftFrontMotor.setMotorDirection(0, iSpeed);
    this->orightBackMotor.setMotorDirection(0, iSpeed);
}

void movementController::rotationLeft(int iSpeed){
    this->orightFrontMotor.setMotorDirection(0, iSpeed);
    this->oleftFrontMotor.setMotorDirection(1, iSpeed);
    this->orightBackMotor.setMotorDirection(0,iSpeed);
    this->oleftBackMotor.setMotorDirection(1, iSpeed);
}

void movementController::rotationRight(int iSpeed){
    this->orightFrontMotor.setMotorDirection(0, iSpeed);
    this->oleftFrontMotor.setMotorDirection(1, iSpeed);
    this->orightBackMotor.setMotorDirection(0,iSpeed);
    this->oleftBackMotor.setMotorDirection(1, iSpeed);
}

  
void movementController::movementstop(){  
    this->orightBackMotor.stopMotor();
    this->oleftBackMotor.stopMotor();
    this->orightFrontMotor.stopMotor();
    this->oleftFrontMotor.stopMotor();
}