#ifndef MECANUM_ODOMETRIE_H
#define MECANUM_ODOMETRIE_H

#include <Arduino.h>
#include <math.h>

// Structure pour stocker la pose du robot (position + orientation)
typedef struct {
    float x;          // Position X en mètres
    float y;          // Position Y en mètres  
    float theta;      // Orientation en radians
} RobotPose_t;

// Structure pour les vitesses du robot
typedef struct {
    float vx;         // Vitesse linéaire X (m/s)
    float vy;         // Vitesse linéaire Y (m/s)
    float omega;      // Vitesse angulaire (rad/s)
} RobotVelocity_t;

// Structure principale de l'odométrie
typedef struct {
    RobotPose_t pose;              // Pose actuelle du robot
    RobotVelocity_t velocity;      // Vitesses actuelles
    float wheelRadius;             // Rayon des roues (m)
    float wheelBase;               // Distance entre roues avant/arrière (m)
    float trackWidth;              // Distance entre roues gauche/droite (m)
    unsigned long lastUpdateTime;  // Timestamp de la dernière mise à jour
    bool initialized;              // Flag d'initialisation
} MecanumOdometry_t;

// Fonctions d'initialisation
void MecanumOdometry_init(MecanumOdometry_t* odometry, float wheelRadius, float wheelBase, float trackWidth);
void MecanumOdometry_reset(MecanumOdometry_t* odometry);
void MecanumOdometry_setPose(MecanumOdometry_t* odometry, float x, float y, float theta);

// Fonctions de calcul odométrique
void MecanumOdometry_updateWheelSpeeds(MecanumOdometry_t* odometry, float wheelSpeed[4]);
void MecanumOdometry_updateFromWheelVelocities(MecanumOdometry_t* odometry,float wheelVelocities[4],float deltaTime);

// Fonctions de cinématique
void MecanumOdometry_updateKinematics(float wheelVelocities[4],float wheelRadius, float wheelBase, float trackWidth,float* vx, float* vy, float* omega);

// Fonctions utilitaires
float MecanumOdometry_getDistanceTraveled(const MecanumOdometry_t* odometry, float startX, float startY);

// Fonctions de conversion
float MecanumOdometry_normalizeAngle(float angle);
void MecanumOdometry_rpmArrayToMetersPerSecond(float rpmArray[4], float wheelRadius, float velocityArray[4]);

// debug function
void MecanumOdometry_debug(const MecanumOdometry_t* odometry);

#endif // MECANUM_ODOMETRIE_H
