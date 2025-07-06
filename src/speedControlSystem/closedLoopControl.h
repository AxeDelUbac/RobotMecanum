#ifndef CLOSEDLOOPCONTROL_H
#define CLOSEDLOOPCONTROL_H

#include "motorController/motorGearBox.h"
#include <Arduino.h>

class ClosedLoopControl {
public:
    ClosedLoopControl(float Kp = 1.0f, float Ki = 0.1f, float Kd = 0.01f);

    float updatePIDControl(float setpoint, float measuredSpeed);
    void setTunings(float Kp, float Ki, float Kd);

private:
    float thresholdPID(float pwmOutput);

    float Kp_PID;
    float Ki_PID;
    float Kd_PID;

    float lastError;
    float integral;

    // motorGearBox* controlledMotor;
};
#endif