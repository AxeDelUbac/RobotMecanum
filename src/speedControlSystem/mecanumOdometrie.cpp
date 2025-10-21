#include "mecanumOdometrie.h"

// Constantes pour la cinématique mécanum
// Ces valeurs doivent être ajustées selon votre robot
static const float DEFAULT_WHEEL_BASE = 0.20f;    // Distance avant/arrière (m) - à ajuster
static const float DEFAULT_TRACK_WIDTH = 0.18f;   // Distance gauche/droite (m) - à ajuster

/**
 * Initialise le système d'odométrie mécanum
 * @param odometry Pointeur vers la structure d'odométrie
 * @param wheelRadius Rayon des roues en mètres
 * @param wheelBase Distance entre les roues avant et arrière en mètres
 * @param trackWidth Distance entre les roues gauche et droite en mètres
 */
void MecanumOdometry_init(MecanumOdometry_t* odometry, float wheelRadius, float wheelBase, float trackWidth)
{    
    // Initialisation des paramètres géométriques
    odometry->wheelRadius = wheelRadius;
    odometry->wheelBase = wheelBase;
    odometry->trackWidth = trackWidth;
    
    // Reset de la pose et des vitesses
    MecanumOdometry_reset(odometry);
    
    odometry->initialized = true;
    
    Serial.println("Mecanum Odometry initialized");
    Serial.print("Wheel radius: "); Serial.println(wheelRadius, 4);
    Serial.print("Wheel base: "); Serial.println(wheelBase, 4);
    Serial.print("Track width: "); Serial.println(trackWidth, 4);
}

/**
 * Remet à zéro la pose et les vitesses du robot
 */
void MecanumOdometry_reset(MecanumOdometry_t* odometry)
{
    
    // Reset de la pose
    odometry->pose.x = 0.0f;
    odometry->pose.y = 0.0f;
    odometry->pose.theta = 0.0f;
    
    // Reset des vitesses
    odometry->velocity.vx = 0.0f;
    odometry->velocity.vy = 0.0f;
    odometry->velocity.omega = 0.0f;
    
    // Reset du timestamp
    odometry->lastUpdateTime = millis();
    
    Serial.println("Odometry reset to origin");
}

/**
 * Définit la pose actuelle du robot
 */
void MecanumOdometry_setPose(MecanumOdometry_t* odometry, float x, float y, float theta)
{
    odometry->pose.x = x;
    odometry->pose.y = y;
    odometry->pose.theta = MecanumOdometry_normalizeAngle(theta);
    
    Serial.print("Pose set to: X="); Serial.print(x, 3);
    Serial.print(" Y="); Serial.print(y, 3);
    Serial.print(" Theta="); Serial.println(theta, 3);
}

/**
 * Met à jour l'odométrie à partir des vitesses des roues en RPM
 * @param wheelSpeedFL Vitesse roue avant-gauche (RPM)
 * @param wheelSpeedFR Vitesse roue avant-droite (RPM)
 * @param wheelSpeedRL Vitesse roue arrière-gauche (RPM)
 * @param wheelSpeedRR Vitesse roue arrière-droite (RPM)
 */
void MecanumOdometry_updateWheelSpeeds(MecanumOdometry_t* odometry, 
                                          float wheelSpeed[4])
{
    if (!odometry || !odometry->initialized || !wheelSpeed) return;
    
    // Conversion RPM vers vitesse linéaire (m/s) - Version optimisée
    float wheelVelocities[4];  // [FL, FR, RL, RR]
    MecanumOdometry_rpmArrayToMetersPerSecond(wheelSpeed, odometry->wheelRadius, wheelVelocities);
    
    // Calcul du temps écoulé
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - odometry->lastUpdateTime) / 1000.0f; // Conversion en secondes
    
    // Gestion des différents cas de deltaTime
    if (deltaTime < 0.0f) {
        // Cas de débordement de millis() (après ~49 jours)
        Serial.println("Warning: millis() overflow detected, resetting timestamp");
        odometry->lastUpdateTime = currentTime;
        return; // Ignorer cette mise à jour
    }
    
    if (deltaTime > 1.0f) {
        // Cas d'une pause trop longue (> 1 seconde)
        Serial.println("Warning: Large time gap detected, resetting timestamp");
        odometry->lastUpdateTime = currentTime;
        return; // Ignorer cette mise à jour pour éviter les sauts
    }
    
    if (deltaTime > 0.001f) { 
        // Cas normal : mise à jour de l'odométrie
        MecanumOdometry_updateFromWheelVelocities(odometry, wheelVelocities, deltaTime);
        odometry->lastUpdateTime = currentTime;
    }
    // Cas deltaTime trop petit (< 1ms) : on ignore silencieusement pour éviter le bruit
}

/**
 * Met à jour l'odométrie à partir des vitesses des roues en m/s
 */
void MecanumOdometry_updateFromWheelVelocities(MecanumOdometry_t* odometry,
                                              float wheelVelocities[4],
                                              float deltaTime)
{
    if (!odometry || !odometry->initialized || !wheelVelocities || deltaTime <= 0) return;
    
    // Calcul de la cinématique directe (vitesses des roues → vitesses du robot)
    float vx, vy, omega;
    MecanumOdometry_updateKinematics(wheelVelocities, odometry->wheelRadius, odometry->wheelBase,
                                     odometry->trackWidth, &vx, &vy, &omega);
    
    // Mise à jour des vitesses dans la structure
    odometry->velocity.vx = vx;
    odometry->velocity.vy = vy;
    odometry->velocity.omega = omega;
    
    // Intégration pour calculer la nouvelle position
    // Transformation des vitesses du repère robot vers le repère monde
    float cos_theta = cos(odometry->pose.theta);
    float sin_theta = sin(odometry->pose.theta);
    
    float vx_world = vx * cos_theta - vy * sin_theta;
    float vy_world = vx * sin_theta + vy * cos_theta;
    
    // Intégration simple (Euler) - pourrait être améliorée avec Runge-Kutta
    odometry->pose.x += vx_world * deltaTime;
    odometry->pose.y += vy_world * deltaTime;
    odometry->pose.theta += omega * deltaTime;
    
    // Normalisation de l'angle
    odometry->pose.theta = MecanumOdometry_normalizeAngle(odometry->pose.theta);
}

/**
 * Calcule la cinématique directe pour roues mécanum
 * Formules pour la configuration mécanum standard (roues à 45°)
 */
void MecanumOdometry_updateKinematics(float wheelVel[4],
                                      float wheelRadius, float wheelBase, float trackWidth,
                                      float* vx, float* vy, float* omega)
{
    if (!vx || !vy || !omega) return;
    
    // Constantes géométriques
    float lx = wheelBase / 2.0f;   // Demi-distance avant/arrière
    float ly = trackWidth / 2.0f;  // Demi-distance gauche/droite      // Somme des demi-distances
    
    // Facteurs d'échelle précalculés (plus efficace)
    float scale_linear = wheelRadius / 4.0f;           // R/4 pour vx et vy
    float scale_angular = wheelRadius / (4.0f * (lx + ly));  // R/(4*(lx+ly)) pour ω
    
    // Matrice de cinématique directe pour roues mécanum (coefficients sans unités)
    // Ordre des roues: [FL, FR, RL, RR]
    static const int8_t kinematics_matrix[3][4] = {
        // vx:  [ 1,  1,  1,  1] 
        { 1,  1,  1,  1},
        // vy:  [-1,  1,  1, -1]  
        {-1,  1,  1, -1},
        // ω:   [-1,  1, -1,  1]
        {-1,  1, -1,  1}
    };
    
    // Calcul matriciel optimisé avec facteurs d'échelle
    // Calcul direct sans boucle (plus rapide)
    *vx = scale_linear * (wheelVel[0] + wheelVel[1] + wheelVel[2] + wheelVel[3]);
    *vy = scale_linear * (-wheelVel[0] + wheelVel[1] + wheelVel[2] - wheelVel[3]);
    *omega = scale_angular * (-wheelVel[0] + wheelVel[1] - wheelVel[2] + wheelVel[3]);
}

/**
 * Calcule la distance parcourue depuis un point de départ
 */
float MecanumOdometry_getDistanceTraveled(const MecanumOdometry_t* odometry, float startX, float startY)
{
    if (!odometry || !odometry->initialized) return 0.0f;
    
    float dx = odometry->pose.x - startX;
    float dy = odometry->pose.y - startY;
    return sqrt(dx * dx + dy * dy);
}

/**
 * Normalise un angle entre -π et π
 */
float MecanumOdometry_normalizeAngle(float angle)
{
    // Si l'angle est supérieur à π, le ramener dans la plage
    if (angle > M_PI) {
        angle -= 2.0f * M_PI;
    }
    
    // Si l'angle est inférieur à -π, le ramener dans la plage
    if (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
    
    return angle;
}

/**
 * Convertit des RPM en vitesse linéaire (m/s) pour toutes les roues
 * Plus efficace : calcul du facteur de conversion une seule fois
 */
void MecanumOdometry_rpmArrayToMetersPerSecond(float rpmArray[4], float wheelRadius, float velocityArray[4])
{
    if (!rpmArray || !velocityArray) return;
    
    // Facteur de conversion précalculé (plus efficace)
    static const float RPM_TO_RAD_S = 2.0f * M_PI / 60.0f;  // Constante
    float conversion_factor = RPM_TO_RAD_S * wheelRadius;
    
    // Conversion vectorisée (4 roues d'un coup)
    for (int i = 0; i < 4; i++) {
        velocityArray[i] = rpmArray[i] * conversion_factor;
    }
}

/**
 * Affiche les informations d'odométrie sur le port série
 */
void MecanumOdometry_debug(const MecanumOdometry_t* odometry)
{
    if (!odometry || !odometry->initialized) {
        Serial.println("Odometry not initialized");
        return;
    }
    
    Serial.print(">Odometry X:"); Serial.println(odometry->pose.x, 3);
    Serial.print(">Odometry Y:"); Serial.println(odometry->pose.y, 3);
    Serial.print(">Odometry Theta:"); Serial.println(odometry->pose.theta, 3);
    Serial.print(">Velocity Vx:"); Serial.println(odometry->velocity.vx, 3);
    Serial.print(">Velocity Vy:"); Serial.println(odometry->velocity.vy, 3);
    Serial.print(">Velocity Omega:"); Serial.println(odometry->velocity.omega, 3);
}
