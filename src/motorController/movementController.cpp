#include "movementController.h"

void MovementController_init(movementController_t* mc){
    if (!mc) return;
    // initialize the underlying motor gear box structs stored inline
    MotorGearBox_init(&mc->orightBackMotor, rightBackMotorHigh, rightBackMotorLow, rightBackMotorPWM);
    MotorGearBox_init(&mc->oleftBackMotor, leftBackMotorHigh, leftBackMotorLow, leftBackMotorPWM);
    MotorGearBox_init(&mc->orightFrontMotor, rightFrontMotorHigh, rightFrontMotorLow, rightFrontMotorPWM);
    MotorGearBox_init(&mc->oleftFrontMotor, leftFrontMotorHigh, leftFrontMotorLow, leftFrontMotorPWM);
    for (int i=0;i<4;i++) mc->fTabSpeedInPwm[i]=0.0f;
}

void MovementController_setDirection(movementController_t* mc, int Xaxis, int Yaxis, int iSpeed) {
    if (!mc) return;
    if (Xaxis == 0 && Yaxis == 1) { // Avant
    MotorGearBox_setMotorDirectionPWM(&mc->orightFrontMotor, 1, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftFrontMotor, 1, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->orightBackMotor, 1, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftBackMotor, 1, iSpeed);
    } else if (Xaxis == 0 && Yaxis == -1) { // Arrière
    MotorGearBox_setMotorDirectionPWM(&mc->orightFrontMotor, 0, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftFrontMotor, 0, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->orightBackMotor, 0, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftBackMotor, 0, iSpeed);
    } else if (Xaxis == 1 && Yaxis == 0) { // Droite
    MotorGearBox_setMotorDirectionPWM(&mc->orightFrontMotor, 1, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftFrontMotor, 0, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->orightBackMotor, 0, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftBackMotor, 1, iSpeed);
    } else if (Xaxis == -1 && Yaxis == 0) { // Gauche
    MotorGearBox_setMotorDirectionPWM(&mc->orightFrontMotor, 1, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftFrontMotor, 0, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->orightBackMotor, 0, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftBackMotor, 1, iSpeed);
    }
}

void MovementController_setRotation(movementController_t* mc, int iSpeed){
    if (!mc) return;

    int Xaxis = 1; // keep previous semantics unclear in original; default rotate right
    int Yaxis = 1;

    if (Xaxis == 1 && Yaxis == 1) { // Rotation droite
    MotorGearBox_setMotorDirectionPWM(&mc->orightFrontMotor, 1, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftFrontMotor, 0, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->orightBackMotor, 1, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftBackMotor, 0, iSpeed);
    } else if (Xaxis == -1 && Yaxis == -1) { // Rotation gauche
    MotorGearBox_setMotorDirectionPWM(&mc->orightFrontMotor, 0, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftFrontMotor, 1, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->orightBackMotor, 0, iSpeed);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftBackMotor, 1, iSpeed);
    }
}

void MovementController_movementFront(movementController_t* mc){
    if (!mc) return;
    MotorGearBox_setMotorDirectionPWM(&mc->oleftFrontMotor, 1, mc->fTabSpeedInPwm[0]);
    MotorGearBox_setMotorDirectionPWM(&mc->orightFrontMotor, 1, mc->fTabSpeedInPwm[1]);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftBackMotor, 1, mc->fTabSpeedInPwm[2]);
    MotorGearBox_setMotorDirectionPWM(&mc->orightBackMotor, 1, mc->fTabSpeedInPwm[3]);
}

void MovementController_movementBack(movementController_t* mc){
    if (!mc) return;
    MotorGearBox_setMotorDirectionPWM(&mc->oleftFrontMotor, 0, mc->fTabSpeedInPwm[0]);
    MotorGearBox_setMotorDirectionPWM(&mc->orightFrontMotor, 0, mc->fTabSpeedInPwm[1]);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftBackMotor, 0, mc->fTabSpeedInPwm[2]);
    MotorGearBox_setMotorDirectionPWM(&mc->orightBackMotor, 0, mc->fTabSpeedInPwm[3]);
}

void MovementController_movementLeft(movementController_t* mc){
    if (!mc) return;
    MotorGearBox_setMotorDirectionPWM(&mc->oleftFrontMotor, 1, mc->fTabSpeedInPwm[0]);
    MotorGearBox_setMotorDirectionPWM(&mc->orightFrontMotor, 0, mc->fTabSpeedInPwm[1]);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftBackMotor, 0, mc->fTabSpeedInPwm[2]);
    MotorGearBox_setMotorDirectionPWM(&mc->orightBackMotor, 1, mc->fTabSpeedInPwm[3]);
}

void MovementController_movementRight(movementController_t* mc){
    if (!mc) return;
    MotorGearBox_setMotorDirectionPWM(&mc->oleftFrontMotor, 0, mc->fTabSpeedInPwm[0]);
    MotorGearBox_setMotorDirectionPWM(&mc->orightFrontMotor, 1, mc->fTabSpeedInPwm[1]);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftBackMotor, 1, mc->fTabSpeedInPwm[2]);
    MotorGearBox_setMotorDirectionPWM(&mc->orightBackMotor, 0, mc->fTabSpeedInPwm[3]);
}


void MovementController_movementFrontLeft(movementController_t* mc){
    if (!mc) return;
    MotorGearBox_setMotorDirectionPWM(&mc->orightFrontMotor, 1, mc->fTabSpeedInPwm[1]);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftBackMotor, 1, mc->fTabSpeedInPwm[2]);
}

void MovementController_movementFrontRight(movementController_t* mc){
    if (!mc) return;
    MotorGearBox_setMotorDirectionPWM(&mc->oleftFrontMotor, 1, mc->fTabSpeedInPwm[0]);
    MotorGearBox_setMotorDirectionPWM(&mc->orightBackMotor, 1, mc->fTabSpeedInPwm[3]);
}
  
void MovementController_movementstop(movementController_t* mc){  
    if (!mc) return;
    MotorGearBox_stopMotor(&mc->orightBackMotor);
    MotorGearBox_stopMotor(&mc->oleftBackMotor);
    MotorGearBox_stopMotor(&mc->orightFrontMotor);
    MotorGearBox_stopMotor(&mc->oleftFrontMotor);
}

void MovementController_setMotorSpeedInPWM(movementController_t* mc, float fTabSpeed[4])
{
    if (!mc) return;
    for (int i = 0; i < 4; i++) {
        mc->fTabSpeedInPwm[i] = fTabSpeed[i];
    }
}