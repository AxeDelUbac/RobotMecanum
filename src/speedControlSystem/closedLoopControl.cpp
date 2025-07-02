#include "closedLoopControl.h"

ClosedLoopControl::ClosedLoopControl(motorGearBox* motor, float Kp, float Ki, float Kd)
    : controlledMotor(motor), Kp_PID(Kp), Ki_PID(Ki), Kd_PID(Kd), lastError(0.0f), integral(0.0f)
{}

float ClosedLoopControl::compute(float setpoint, float measuredSpeed)
{
    float error = setpoint - measuredSpeed;
    integral += Ki_PID * error;
    float derivative = error - lastError;

    float output = Kp_PID * error + integral + Kd_PID * derivative;
    output = thresholdPID(output);

    if (controlledMotor) {
        controlledMotor->setMotorDirection(1, output);
    }

    Serial.print(">Sortie:");
    Serial.println(output);
    Serial.print(">Erreur:");
    Serial.println(error);

    lastError = error;
    return output;
}

void ClosedLoopControl::setTunings(float Kp, float Ki, float Kd)
{
    Kp_PID = Kp;
    Ki_PID = Ki;
    Kd_PID = Kd;
}

float ClosedLoopControl::thresholdPID(float pwmOutput)
{
    if (pwmOutput > 255.0f)
    {
        pwmOutput = 255.0f;
    }
    if (pwmOutput < 0.0f)
    {
        pwmOutput = 0.0f;
    }
    return pwmOutput;
}