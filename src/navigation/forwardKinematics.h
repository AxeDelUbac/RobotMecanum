#ifndef FORWARDKINEMATICS_H
#define FORWARDKINEMATICS_H

#include <Arduino.h>

void ForwardKinematics_computeWheelVelocities(int xAxis, int yAxis, int rotation, float motorSpeeds[4]);
void ForwardKinematics_normalizeMotorsSpeeds(float motorSpeeds[4], float normalisedMotorSpeeds[4] ,float maxValue);

#endif