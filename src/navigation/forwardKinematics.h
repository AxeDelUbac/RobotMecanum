#ifndef FORWARDKINEMATICS_H
#define FORWARDKINEMATICS_H

#include <Arduino.h>

typedef struct {
    int vx;      // Vitesse longitudinale (m/s)
    int vy;      // Vitesse latérale (m/s)
    int omega;   // Vitesse angulaire (rad/s)
    float motorSpeeds[4]; // Vitesses des roues [FL, FR, RL, RR]
    float normalisedMotorSpeeds[4]; // Vitesses normalisées des roues [FL, FR, RL, RR]
    float geometricFactor; // Facteur géométrique (L + W)
} forwardKinematics_t;

void ForwardKinematics_computeWheelVelocities(forwardKinematics_t* fk, float normalisedSpeeds[4]);
void ForwardKinematics_normalizeMotorsSpeeds(forwardKinematics_t* fk ,float maxValue);

void ForwardKinematics_debug(forwardKinematics_t* fk);

#endif