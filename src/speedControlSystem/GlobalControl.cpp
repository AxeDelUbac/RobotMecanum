#include "GlobalControl.h"

GlobalControl::GlobalControl() {
    closedLoopFrontLeft = new ClosedLoopControl(5.0f, 1.0f, 0.05f);
    closedLoopFrontRight = new ClosedLoopControl(5.0f, 1.0f, 0.05f);
    closedLoopRearLeft = new ClosedLoopControl(5.0f, 1.0f, 0.05f);
    closedLoopRearRight = new ClosedLoopControl(5.0f, 1.0f, 0.05f);
}

void GlobalControl::UpdateSetpoint(float fSetpointKmh, float fMeasuredSpeedKmh[4], float fOutputKmh[4] ) {

    fOutputKmh[0] = closedLoopFrontLeft->updatePIDControl(fSetpointKmh, fMeasuredSpeedKmh[0]);
    fOutputKmh[1] = closedLoopFrontRight->updatePIDControl(fSetpointKmh, fMeasuredSpeedKmh[1]);
    fOutputKmh[2] = closedLoopRearLeft->updatePIDControl(fSetpointKmh, fMeasuredSpeedKmh[2]);
    fOutputKmh[3] = closedLoopRearRight->updatePIDControl(fSetpointKmh, fMeasuredSpeedKmh[3]);
}

void GlobalControl::SerialDebug(void) 
{
    Serial.print(">FrontLeft Error :");
    Serial.println(closedLoopFrontLeft->getErrorPID());
    Serial.print(">FrontLeft Output :");
    Serial.println(closedLoopFrontLeft->getOutputPID());

    Serial.print(">FrontRight Error :");
    Serial.println(closedLoopFrontRight->getErrorPID());
    Serial.print(">FrontRight Output :");
    Serial.println(closedLoopFrontRight->getOutputPID());

    Serial.print(">RearLeft Error :");
    Serial.println(closedLoopRearLeft->getErrorPID());
    Serial.print(">RearLeft Output :");
    Serial.println(closedLoopRearLeft->getOutputPID());

    Serial.print(">RearRight Error :");
    Serial.println(closedLoopRearRight->getErrorPID());
    Serial.print(">RearRight Output :");
    Serial.println(closedLoopRearRight->getOutputPID());
}