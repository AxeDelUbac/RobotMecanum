#include "GlobalControl.h"
#include <Arduino.h>

void GlobalControl_init(GlobalControl* gc) {

    ClosedLoopControl_init(&gc->closedLoopFrontLeft, 2.4f, 1.13f, 0.00f);
    ClosedLoopControl_init(&gc->closedLoopFrontRight, 2.4f, 1.13f, 0.05f);
    ClosedLoopControl_init(&gc->closedLoopRearLeft, 2.4f, 1.13f, 0.05f);
    ClosedLoopControl_init(&gc->closedLoopRearRight, 2.4f, 1.13f, 0.05f);
    for (int i = 0; i < 4; i++) gc->ftabSetpointKmh[i] = 0.0f;
}

int iloop = 50;

void GlobalControl_UpdateSetpoint(GlobalControl* gc, float fSetpointKmh[4], float fMeasuredSpeedKmh[4], float fOutputKmh[4]) {
    fOutputKmh[0] = ClosedLoopControl_updatePIDControl(&gc->closedLoopFrontLeft, fSetpointKmh[0], fMeasuredSpeedKmh[0]);
    fOutputKmh[1] = ClosedLoopControl_updatePIDControl(&gc->closedLoopFrontRight, fSetpointKmh[1], fMeasuredSpeedKmh[1]);
    fOutputKmh[2] = ClosedLoopControl_updatePIDControl(&gc->closedLoopRearLeft, fSetpointKmh[2], fMeasuredSpeedKmh[2]);
    fOutputKmh[3] = ClosedLoopControl_updatePIDControl(&gc->closedLoopRearRight, fSetpointKmh[3], fMeasuredSpeedKmh[3]);
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