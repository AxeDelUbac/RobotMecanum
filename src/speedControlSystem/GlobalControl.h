#ifndef GLOBALCONTROL_H
#define GLOBALCONTROL_H

#include "closedLoopControl.h"

class GlobalControl {
public:
    GlobalControl();
    void UpdateSetpoint(float fSetpointKmh, float fMeasuredSpeedKmh[4],float ftabSetpointKmh[4]);
    void SerialDebug(void);

private:
    float ftabSetpointKmh[4]= {0.0f, 0.0f, 0.0f, 0.0f};

    ClosedLoopControl* closedLoopFrontLeft;
    ClosedLoopControl* closedLoopFrontRight;
    ClosedLoopControl* closedLoopRearLeft;
    ClosedLoopControl* closedLoopRearRight;
};

#endif