#include "closedLoopControl.h"

ClosedLoopControl::ClosedLoopControl(float Kp, float Ki, float Kd)
    : Kp_PID(Kp), Ki_PID(Ki), Kd_PID(Kd), lastError(0.0f), integral(0.0f)
{}

float ClosedLoopControl::updatePIDControl(float fSetpointKmh, float fMeasuredSpeedKmh)
{
    float fErrorPID = fSetpointKmh - fMeasuredSpeedKmh;
    integral += Ki_PID * fErrorPID;
    float derivative = fErrorPID - lastError;

    float fOutputPID = Kp_PID * fErrorPID + integral + Kd_PID * derivative;
    fOutputPID = thresholdPID(fOutputPID);

    // if (controlledMotor) {
    //     controlledMotor->setMotorDirection(1, output);
    // }

    Serial.print(">Sortie:");
    Serial.println(fOutputPID);
    Serial.print(">Erreur:");
    Serial.println(fErrorPID);

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