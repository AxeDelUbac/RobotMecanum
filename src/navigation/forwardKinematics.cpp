#include "forwardKinematics.h"

/**
 * @brief Convertit les commandes de déplacement 3 axes en vitesses des 4 roues mecanum
 * 
 * Applique la matrice de transformation cinématique directe pour roues mecanum :
 * [FL]   [ 1  -1  -(L+W)]   [vx]
 * [FR] = [ 1   1   (L+W)] * [vy]
 * [RL]   [ 1   1  -(L+W)]   [ω ]
 * [RR]   [ 1  -1   (L+W)]
 * 
 * @param[in] fk Structure contenant vx, vy, omega (-100 à +100)
 * @param[out] normalisedSpeeds Vitesses calculées [FL, FR, RL, RR]
 * 
 * @note Les entrées sont automatiquement limitées à [-100, +100]
 * @warning Les sorties peuvent dépasser ces limites et nécessiter une normalisation
 */
void ForwardKinematics_computeWheelVelocities(forwardKinematics_t* fk, float normalisedSpeeds[4]) {
    
    // Clamping des valeurs d'entrée
    if(fk->vx > 100) fk->vx = 100;
    else if (fk->vx < -100) fk->vx = -100;
    if(fk->vy > 100) fk->vy = 100;
    else if (fk->vy < -100) fk->vy = -100;
    if(fk->omega > 100) fk->omega = 100;
    else if (fk->omega < -100) fk->omega = -100;
    
    // Application de la matrice de pilotage mecanum
    fk->motorSpeeds[0] = fk->vx - fk->vy - (fk->geometricFactor * fk->omega);  // Front Left  (FL)
    fk->motorSpeeds[1] = fk->vx + fk->vy + (fk->geometricFactor * fk->omega);  // Front Right (FR)  
    fk->motorSpeeds[2] = fk->vx + fk->vy - (fk->geometricFactor * fk->omega);  // Rear Left   (RL)
    fk->motorSpeeds[3] = fk->vx - fk->vy + (fk->geometricFactor * fk->omega);  // Rear Right  (RR)

    ForwardKinematics_normalizeMotorsSpeeds(fk ,100.0f);

    normalisedSpeeds[0] = fk->normalisedMotorSpeeds[0];
    normalisedSpeeds[1] = fk->normalisedMotorSpeeds[1];
    normalisedSpeeds[2] = fk->normalisedMotorSpeeds[2];
    normalisedSpeeds[3] = fk->normalisedMotorSpeeds[3];
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
void ForwardKinematics_normalizeMotorsSpeeds(forwardKinematics_t* fk ,float maxValue) {
    // Trouve la valeur absolue maximale
    float maximalMotorSpeed = 0.0f;
    float absoluteSpeed;
    float scaleFactor;
    
    for (int i = 0; i < 4; i++) {
        // Calcul de la valeur absolue
        if (fk->motorSpeeds[i] < 0) {
            absoluteSpeed = -fk->motorSpeeds[i];
        } else {
            absoluteSpeed = fk->motorSpeeds[i];
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
            fk->normalisedMotorSpeeds[i] = fk->motorSpeeds[i] * scaleFactor;
        }
    }
    else {
    // Copie directement les valeurs si pas de normalisation
        for (int i = 0; i < 4; i++) {
            fk->normalisedMotorSpeeds[i] = fk->motorSpeeds[i];
        }
    }
}

void ForwardKinematics_debug(forwardKinematics_t* fk){
   
    Serial.print(">Odometry X en m:"); Serial.println(fk->vx);
    Serial.print(">Odometry Y en m:"); Serial.println(fk->vy);
    Serial.print(">Odometry Theta en rad:"); Serial.println(fk->omega);
    for (int i = 0; i < 4; i++) {
        Serial.print(">Motor Speed "); 
        Serial.print(i); Serial.print(" :"); 
        Serial.println(fk->motorSpeeds[i]);
    }
    for (int i = 0; i < 4; i++) {
        Serial.print(">Normalised Motor Speed "); 
        Serial.print(i); Serial.print(" :"); 
        Serial.println(fk->normalisedMotorSpeeds[i]);
    }
}