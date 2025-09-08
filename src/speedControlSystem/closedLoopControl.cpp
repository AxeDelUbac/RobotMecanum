#include "closedLoopControl.h"

ClosedLoopControl::ClosedLoopControl(float Kp, float Ki, float Kd)
    : Kp_PID(Kp), Ki_PID(Ki), Kd_PID(Kd), lastError(0.0f), integral(0.0f)
{}

float ClosedLoopControl::updatePIDControl(float fSetpointRpm, float fMeasuredSpeedRpm)
{
    // Erreur en RPM
    fErrorPID = fSetpointRpm - fMeasuredSpeedRpm;

    // Intégrale (anti-windup)
    integral += fErrorPID;
    if (integral > 100.0f) integral = 100.0f;
    if (integral < -100.0f) integral = -100.0f;

    // Dérivée
    float derivative = fErrorPID - lastError;

    // Calcul PID (sortie en PWM 8 bits)
    fOutputPID = Kp_PID * fErrorPID + Ki_PID * integral + Kd_PID * derivative;

    // Saturation PWM 8 bits
    fOutputPID = thresholdPID(fOutputPID);

    lastError = fErrorPID;

    return fOutputPID;
}

void ClosedLoopControl::setTunings(float Kp, float Ki, float Kd)
{
    Kp_PID = Kp;
    Ki_PID = Ki;
    Kd_PID = Kd;
}

float ClosedLoopControl::thresholdPID(float fPwmOutput)
{
    if (fPwmOutput > 255.0f)
    {
        fPwmOutput = 255.0f;
    }
    if (fPwmOutput < 0.0f)
    {
        fPwmOutput = 0.0f;
    }
    return fPwmOutput;
}

float ClosedLoopControl::getOutputPID(void)
{
    return fOutputPID;
}

float ClosedLoopControl::getErrorPID(void)
{
    return fErrorPID;
}