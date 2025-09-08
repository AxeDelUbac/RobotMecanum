#include "GlobalControl.h"
#include <Arduino.h>

void GlobalControl_init(GlobalControl* gc) {
    new (&gc->closedLoopFrontLeft) ClosedLoopControl(5.0f, 1.0f, 0.05f);
    new (&gc->closedLoopFrontRight) ClosedLoopControl(5.0f, 1.0f, 0.05f);
    new (&gc->closedLoopRearLeft) ClosedLoopControl(5.0f, 1.0f, 0.05f);
    new (&gc->closedLoopRearRight) ClosedLoopControl(5.0f, 1.0f, 0.05f);
    for (int i = 0; i < 4; i++) gc->ftabSetpointKmh[i] = 0.0f;
}

void GlobalControl_UpdateSetpoint(GlobalControl* gc, float fSetpointKmh, float fMeasuredSpeedKmh[4], float fOutputKmh[4]) {
    fOutputKmh[0] = gc->closedLoopFrontLeft.updatePIDControl(fSetpointKmh, fMeasuredSpeedKmh[0]);
    fOutputKmh[1] = gc->closedLoopFrontRight.updatePIDControl(fSetpointKmh, fMeasuredSpeedKmh[1]);
    fOutputKmh[2] = gc->closedLoopRearLeft.updatePIDControl(fSetpointKmh, fMeasuredSpeedKmh[2]);
    fOutputKmh[3] = gc->closedLoopRearRight.updatePIDControl(fSetpointKmh, fMeasuredSpeedKmh[3]);
}

void GlobalControl_SerialDebug(GlobalControl* gc) {
    Serial.print(">FrontLeft Error :");
    Serial.println(gc->closedLoopFrontLeft.getErrorPID());
    Serial.print(">FrontLeft Output :");
    Serial.println(gc->closedLoopFrontLeft.getOutputPID());

    Serial.print(">FrontRight Error :");
    Serial.println(gc->closedLoopFrontRight.getErrorPID());
    Serial.print(">FrontRight Output :");
    Serial.println(gc->closedLoopFrontRight.getOutputPID());

    Serial.print(">RearLeft Error :");
    Serial.println(gc->closedLoopRearLeft.getErrorPID());
    Serial.print(">RearLeft Output :");
    Serial.println(gc->closedLoopRearLeft.getOutputPID());

    Serial.print(">RearRight Error :");
    Serial.println(gc->closedLoopRearRight.getErrorPID());
    Serial.print(">RearRight Output :");
    Serial.println(gc->closedLoopRearRight.getOutputPID());
}