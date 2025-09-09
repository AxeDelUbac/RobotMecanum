#include "closedLoopControl.h"
#include <Arduino.h>

void ClosedLoopControl_init(ClosedLoopControl* ctl, float Kp, float Ki, float Kd) {
    if (!ctl) return;
    ctl->Kp = Kp;
    ctl->Ki = Ki;
    ctl->Kd = Kd;
    ctl->lastError = 0.0f;
    ctl->integral = 0.0f;
    ctl->prevDerivativeFiltered = 0.0f;
    ctl->fOutputPID = 0.0f;
    ctl->fErrorPID = 0.0f;
}

float ClosedLoopControl_updatePIDControl(ClosedLoopControl* ctl, float setpoint, float measuredSpeed) {
    if (!ctl){
        return 0.0f;
    }

    // Erreur
    ctl->fErrorPID = setpoint - measuredSpeed;

    // Intégrale (anti-windup)
    ctl->integral += ctl->fErrorPID;
    if (ctl->integral > 100.0f) ctl->integral = 100.0f;
    if (ctl->integral < -100.0f) ctl->integral = -100.0f;

    // Dérivée (raw)
    float derivative = ctl->fErrorPID - ctl->lastError;

    // Calcul PID
    ctl->fOutputPID = ctl->Kp * ctl->fErrorPID + ctl->Ki * ctl->integral + ctl->Kd * derivative;

    // Saturation
    ctl->fOutputPID = ClosedLoopControl_thresholdPID(ctl, ctl->fOutputPID);

    ctl->lastError = ctl->fErrorPID;

    Serial.print(" - output: ");
    Serial.println(ctl->fOutputPID);
    return ctl->fOutputPID;
}

void ClosedLoopControl_setTunings(ClosedLoopControl* ctl, float Kp, float Ki, float Kd) {
    if (!ctl) return;
    ctl->Kp = Kp;
    ctl->Ki = Ki;
    ctl->Kd = Kd;
}

float ClosedLoopControl_getOutputPID(ClosedLoopControl* ctl) {
    return ctl ? ctl->fOutputPID : 0.0f;
}

float ClosedLoopControl_getErrorPID(ClosedLoopControl* ctl) {
    return ctl ? ctl->fErrorPID : 0.0f;
}

float ClosedLoopControl_thresholdPID(ClosedLoopControl* ctl, float pwmOutput) {
    (void)ctl;
    if (pwmOutput > 255.0f) pwmOutput = 255.0f;
    if (pwmOutput < 0.0f) pwmOutput = 0.0f;
    return pwmOutput;
}
