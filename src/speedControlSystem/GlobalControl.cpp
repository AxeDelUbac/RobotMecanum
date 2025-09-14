#include "GlobalControl.h"
#include <Arduino.h>

void GlobalControl_init(GlobalControl* gc) {
    ClosedLoopControl_init(&gc->closedLoopFrontLeft, 1.5f, 0.1f, 0.04f);
    ClosedLoopControl_init(&gc->closedLoopFrontRight, 1.0f, 0.01f, 0.01f);
    ClosedLoopControl_init(&gc->closedLoopRearLeft, 1.0f, 0.01f, 0.01f);
    ClosedLoopControl_init(&gc->closedLoopRearRight, 1.0f, 0.1f, 0.01f);
    for (int i = 0; i < 4; i++) gc->ftabSetpointKmh[i] = 0.0f;
}

void GlobalControl_UpdateSetpoint(GlobalControl* gc, float fSetpointKmh, float fMeasuredSpeedKmh[4], float fOutputKmh[4]) {
    fOutputKmh[0] = ClosedLoopControl_updatePIDControl(&gc->closedLoopFrontLeft, fSetpointKmh, fMeasuredSpeedKmh[0]);
    fOutputKmh[1] = ClosedLoopControl_updatePIDControl(&gc->closedLoopFrontRight, fSetpointKmh, fMeasuredSpeedKmh[1]);
    fOutputKmh[2] = ClosedLoopControl_updatePIDControl(&gc->closedLoopRearLeft, fSetpointKmh, fMeasuredSpeedKmh[2]);
    fOutputKmh[3] = ClosedLoopControl_updatePIDControl(&gc->closedLoopRearRight, fSetpointKmh, fMeasuredSpeedKmh[3]);
    
}

void GlobalControl_SerialDebug(GlobalControl* gc) {
    Serial.print(">FrontLeft Error :");
    Serial.println(ClosedLoopControl_getErrorPID(&gc->closedLoopFrontLeft));
    Serial.print(">FrontLeft Output :");
    Serial.println(ClosedLoopControl_getOutputPID(&gc->closedLoopFrontLeft));

    Serial.print(">FrontRight Error :");
    Serial.println(ClosedLoopControl_getErrorPID(&gc->closedLoopFrontRight));
    Serial.print(">FrontRight Output :");
    Serial.println(ClosedLoopControl_getOutputPID(&gc->closedLoopFrontRight));

    Serial.print(">RearLeft Error :");
    Serial.println(ClosedLoopControl_getErrorPID(&gc->closedLoopRearLeft));
    Serial.print(">RearLeft Output :");
    Serial.println(ClosedLoopControl_getOutputPID(&gc->closedLoopRearLeft));

    Serial.print(">RearRight Error :");
    Serial.println(ClosedLoopControl_getErrorPID(&gc->closedLoopRearRight));
    Serial.print(">RearRight Output :");
    Serial.println(ClosedLoopControl_getOutputPID(&gc->closedLoopRearRight));
}