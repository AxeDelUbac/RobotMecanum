#include "movementController.h"
#include <Arduino.h>



movementController::movementController()
    : 
    leftFrontMotor(leftFrontMotorHigh, leftFrontMotorLow, leftFrontMotorPWM),
    rightFrontMotor(rightFrontMotorHigh, rightFrontMotorLow, rightFrontMotorPWM),
    leftBackMotor(leftBackMotorHigh, leftBackMotorLow, leftBackMotorPWM),
    rightBackMotor(rightBackMotorHigh, rightBackMotorLow, rightBackMotorPWM)
    {}

void movementController::setDirection(int Xaxis, int Yaxis, int iSpeed){
    if (Xaxis == 0 && Yaxis == 1) { // Avant
        this->rightFrontMotor.setMotorDirection(1, iSpeed);
        this->leftFrontMotor.setMotorDirection(1, iSpeed);
        this->rightBackMotor.setMotorDirection(1,iSpeed);
        this->leftBackMotor.setMotorDirection(1,iSpeed);
    } else if (Xaxis == 0 && Yaxis == -1) { // Arrière
        this->rightFrontMotor.setMotorDirection(0, iSpeed);
        this->leftFrontMotor.setMotorDirection(0, iSpeed);
        this->rightBackMotor.setMotorDirection(0,iSpeed);
        this->leftBackMotor.setMotorDirection(0, iSpeed);
    } else if (Xaxis == 1 && Yaxis == 0) { // Droite
        this->rightFrontMotor.setMotorDirection(1, iSpeed);
        this->leftFrontMotor.setMotorDirection(0, iSpeed);
        this->rightBackMotor.setMotorDirection(0,iSpeed);
        this->leftBackMotor.setMotorDirection(1, iSpeed);
    } else if (Xaxis == -1 && Yaxis == 0) { // Gauche
        this->rightFrontMotor.setMotorDirection(1, iSpeed);
        this->leftFrontMotor.setMotorDirection(0, iSpeed);
        this->rightBackMotor.setMotorDirection(0,iSpeed);
        this->leftBackMotor.setMotorDirection(1, iSpeed);
    }
}

void movementController::setRotation(int iSpeed){

    int Xaxis; // Exemple de valeur pour Xaxis
    int Yaxis; // Exemple de valeur pour Yaxis

    if (Xaxis == 1 && Yaxis == 1) { // Rotation droite
        this->rightFrontMotor.setMotorDirection(1, iSpeed);
        this->leftFrontMotor.setMotorDirection(0, iSpeed);
        this->rightBackMotor.setMotorDirection(1,iSpeed);
        this->leftBackMotor.setMotorDirection(0, iSpeed);
    } else if (Xaxis == -1 && Yaxis == -1) { // Rotation gauche
        this->rightFrontMotor.setMotorDirection(0, iSpeed);
        this->leftFrontMotor.setMotorDirection(1, iSpeed);
        this->rightBackMotor.setMotorDirection(0,iSpeed);
        this->leftBackMotor.setMotorDirection(1, iSpeed);
    }
}

void movementController::movementFront(int iSpeed){
    this->rightFrontMotor.setMotorDirection(1, iSpeed);
    this->leftFrontMotor.setMotorDirection(1, iSpeed);
    this->rightBackMotor.setMotorDirection(1,iSpeed);
    this->leftBackMotor.setMotorDirection(1, iSpeed);
}

void movementController::movementBack(int iSpeed){
    this->rightFrontMotor.setMotorDirection(0, iSpeed);
    this->leftFrontMotor.setMotorDirection(0, iSpeed);
    this->rightBackMotor.setMotorDirection(0,iSpeed);
    this->leftBackMotor.setMotorDirection(0, iSpeed);
}

void movementController::movementLeft(int iSpeed){
    this->rightFrontMotor.setMotorDirection(1, iSpeed);
    this->leftFrontMotor.setMotorDirection(0, iSpeed);
    this->rightBackMotor.setMotorDirection(0,iSpeed);
    this->leftBackMotor.setMotorDirection(1, iSpeed);
}

void movementController::movementRight(int iSpeed){
    this->rightFrontMotor.setMotorDirection(0, iSpeed);
    this->leftFrontMotor.setMotorDirection(1, iSpeed);
    this->rightBackMotor.setMotorDirection(1,iSpeed);
    this->leftBackMotor.setMotorDirection(0, iSpeed);
}


void movementController::movementFrontLeft(int iSpeed){
    this->rightFrontMotor.setMotorDirection(1, iSpeed);
    this->leftBackMotor.setMotorDirection(1, iSpeed);
}

void movementController::movementFrontRight(int iSpeed){
    this->leftFrontMotor.setMotorDirection(1, iSpeed);
    this->rightBackMotor.setMotorDirection(1, iSpeed);
}


void movementController::movementBackLeft(int iSpeed){
    this->leftFrontMotor.setMotorDirection(0, iSpeed);
    this->rightBackMotor.setMotorDirection(0, iSpeed);
}

void movementController::movementBackRight(int iSpeed){
    this->leftFrontMotor.setMotorDirection(0, iSpeed);
    this->rightBackMotor.setMotorDirection(0, iSpeed);
}

void movementController::rotationLeft(int iSpeed){
    this->rightFrontMotor.setMotorDirection(0, iSpeed);
    this->leftFrontMotor.setMotorDirection(1, iSpeed);
    this->rightBackMotor.setMotorDirection(0,iSpeed);
    this->leftBackMotor.setMotorDirection(1, iSpeed);
}

void movementController::rotationRight(int iSpeed){
    this->rightFrontMotor.setMotorDirection(0, iSpeed);
    this->leftFrontMotor.setMotorDirection(1, iSpeed);
    this->rightBackMotor.setMotorDirection(0,iSpeed);
    this->leftBackMotor.setMotorDirection(1, iSpeed);
}

  
void movementController::movementstop(){  
    this->rightBackMotor.stopMotor();
    this->leftBackMotor.stopMotor();
    this->rightFrontMotor.stopMotor();
    this->leftFrontMotor.stopMotor();
}