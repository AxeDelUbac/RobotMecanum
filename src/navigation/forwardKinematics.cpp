#include "forwardKinematics.h"

float geometricFactor = 1.0f;

/**
 * @brief Convertit les commandes de déplacement 3 axes en vitesses des 4 roues mecanum
 * 
 * Applique la matrice de transformation cinématique directe pour roues mecanum :
 * [FL]   [ 1  -1  -(L+W)]   [vx]
 * [FR] = [ 1   1   (L+W)] * [vy]
 * [RL]   [ 1   1  -(L+W)]   [ω ]
 * [RR]   [ 1  -1   (L+W)]
 * 
 * @param[in] vx Vitesse latérale (-100 à +100), positif = droite
 * @param[in] vy Vitesse longitudinale (-100 à +100), positif = avant
 * @param[in] w Vitesse de rotation (-100 à +100), positif = horaire
 * @param[out] motorSpeeds Vitesses calculées [FL, FR, RL, RR]
 * 
 * @note Les entrées sont automatiquement limitées à [-100, +100]
 * @warning Les sorties peuvent dépasser ces limites et nécessiter une normalisation
 */
void ForwardKinematics_computeWheelVelocities(int vx, int vy, int w, float motorSpeeds[4]) {
    
    // Clamping des valeurs d'entrée
    if(vx > 100) vx = 100;
    else if (vx < -100) vx = -100;

    if(vy > 100) vy = 100;
    else if (vy < -100) vy = -100;

    if(w > 100) w = 100;
    else if (w < -100) w = -100;
    
    // Application de la matrice de pilotage mecanum
    motorSpeeds[0] = vx - vy - (geometricFactor * w);  // Front Left  (FL)
    motorSpeeds[1] = vx + vy + (geometricFactor * w);  // Front Right (FR)  
    motorSpeeds[2] = vx + vy - (geometricFactor * w);  // Rear Left   (RL)
    motorSpeeds[3] = vx - vy + (geometricFactor * w);  // Rear Right  (RR)
}

/**
 * @brief Normalise les vitesses moteur pour éviter la saturation
 * 
 * Applique un facteur d'échelle uniforme si une vitesse dépasse maxValue.
 * Préserve les proportions entre moteurs.
 * 
 * @param[in] motorSpeeds Vitesses d'entrée [FL, FR, RL, RR]
 * @param[out] normalisedMotorSpeeds Vitesses normalisées de sortie
 * @param[in] maxValue Limite maximale absolue (ex: 100.0f ou 255.0f)
 */
void ForwardKinematics_normalizeMotorsSpeeds(float motorSpeeds[4], float normalisedMotorSpeeds[4] ,float maxValue) {
    // Trouve la valeur absolue maximale
    float maximalMotorSpeed = 0.0f;
    float absoluteSpeed;
    float scaleFactor;
    
    for (int i = 0; i < 4; i++) {
        // Calcul de la valeur absolue
        if (motorSpeeds[i] < 0) {
            absoluteSpeed = -motorSpeeds[i];
        } else {
            absoluteSpeed = motorSpeeds[i];
        }
        // Trouve le maximum
        if (absoluteSpeed > maximalMotorSpeed) {
            maximalMotorSpeed = absoluteSpeed;
        }
    }
    // Normalise si nécessaire en gardant les proportions
    if (maximalMotorSpeed > maxValue) {
        scaleFactor = maxValue / maximalMotorSpeed;
        for (int i = 0; i < 4; i++) {
            normalisedMotorSpeeds[i] = motorSpeeds[i] * scaleFactor;
        }
    }
}