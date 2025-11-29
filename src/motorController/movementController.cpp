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

void MovementController_setMovement(movementController_t* mc, float fMotorSpeeds[4]) {
    if (!mc) return;

    // Definis la direction et la vitesse des moteurs
    for(int i=0; i<4; i++)
    {
      mc->fTabSpeedInPercent[i]=fMotorSpeeds[i];
      if (fMotorSpeeds[i]>0)
      {
        mc->bTabDirection[i]=true;
      }
      else
      {
        mc->bTabDirection[i]=false;
      }

      //normalisation de la vitesse en pourentage 0-100 vers pwm 0-255
      mc->fTabSpeedInPwm[i] = mc->fTabSpeedInPercent[i] * 255 / 100;
    }

    MotorGearBox_setMotorDirectionPWM(&mc->oleftFrontMotor, mc->bTabDirection[0], mc->fTabSpeedInPwm[0]);
    MotorGearBox_setMotorDirectionPWM(&mc->orightFrontMotor, mc->bTabDirection[1], mc->fTabSpeedInPwm[1]);
    MotorGearBox_setMotorDirectionPWM(&mc->oleftBackMotor, mc->bTabDirection[2], mc->fTabSpeedInPwm[2]);
    MotorGearBox_setMotorDirectionPWM(&mc->orightBackMotor, mc->bTabDirection[3], mc->fTabSpeedInPwm[3]);
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