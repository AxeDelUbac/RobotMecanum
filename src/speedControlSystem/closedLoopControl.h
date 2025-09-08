#ifndef CLOSEDLOOPCONTROL_H
#define CLOSEDLOOPCONTROL_H

#include "motorController/motorGearBox.h"
#include <Arduino.h>

class ClosedLoopControl {
public:
    ClosedLoopControl(float Kp = 1.0f, float Ki = 500.0f, float Kd = 0.1f);

    float updatePIDControl(float setpoint, float measuredSpeed);
    void setTunings(float Kp, float Ki, float Kd);
    float getOutputPID(void);
    float getErrorPID(void);
    float thresholdPID(float pwmOutput);

private:

    float Kp_PID;
    float Ki_PID;
    float Kd_PID;

    float lastError;
    float integral;

    float fOutputPID;
    float fErrorPID;

};
#endif