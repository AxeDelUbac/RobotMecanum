#include "mecanumOdometrie.h"

// Constantes pour la cinématique mécanum
// Ces valeurs doivent être ajustées selon votre robot
static const float DEFAULT_WHEEL_BASE = 0.20f;    // Distance avant/arrière (m) - à ajuster
static const float DEFAULT_TRACK_WIDTH = 0.18f;   // Distance gauche/droite (m) - à ajuster

#define RPM_TO_RAD_S (2.0f * M_PI / 60.0f)  // Conversion RPM vers rad/s

unsigned long currentTime = millis();
float deltaTime = 0.0f;

/**
 * @brief Initialise le système d'odométrie mécanum
 * @param odometry Pointeur vers la structure d'odométrie à initialiser
 * @param wheelRadius Rayon des roues en mètres
 * @param wheelBase Distance entre les roues avant et arrière en mètres  
 * @param trackWidth Distance entre les roues gauche et droite en mètres
 * @return Aucun
 * @note Cette fonction appelle automatiquement MecanumOdometry_reset()
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
}

/**
 * @brief Remet à zéro la pose et les vitesses du robot
 * @param odometry Pointeur vers la structure d'odométrie à réinitialiser
 * @return Aucun
 * @note Remet la position à (0,0), l'angle à 0 et toutes les vitesses à zéro
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
    
}

/**
 * @brief Met à jour l'odométrie à partir des vitesses des roues en RPM
 * @param odometry Pointeur vers la structure d'odométrie à mettre à jour
 * @param wheelSpeed Tableau des vitesses des 4 roues [FL, FR, RL, RR] en RPM
 * @return Aucun
 * @note Ordre des roues : [0]=Avant-Gauche, [1]=Avant-Droite, [2]=Arrière-Gauche, [3]=Arrière-Droite
 */
void MecanumOdometry_updateWheelSpeeds(MecanumOdometry_t* odometry, float wheelSpeed[4])
{
    
    // Conversion RPM vers vitesse linéaire (m/s) - Version optimisée
    float wheelVelocities[4];
    MecanumOdometry_rpmArrayToMetersPerSecond(odometry, wheelSpeed, wheelVelocities);
    
    // Calcul du temps écoulé
    currentTime = millis();
    deltaTime = (currentTime - odometry->lastUpdateTime) / 1000.0f; // Conversion en secondes
    
    // Cas normal : mise à jour de l'odométrie
    MecanumOdometry_updatePose(odometry, wheelVelocities, deltaTime);
    odometry->lastUpdateTime = currentTime;
}

/**
 * @brief Met à jour la pose du robot à partir des vitesses des roues en m/s
 * @param odometry Pointeur vers la structure d'odométrie à mettre à jour
 * @param wheelVelocities Tableau des vitesses des 4 roues en m/s [FL, FR, RL, RR]
 * @param deltaTime Intervalle de temps écoulé en secondes depuis la dernière mise à jour
 * @return Aucun
 * @note Calcule les vitesses du robot puis intègre pour obtenir la nouvelle position
 */
void MecanumOdometry_updatePose(MecanumOdometry_t* odometry,float wheelVelocities[4],float deltaTime)
{
    // Calcul de la cinématique directe (vitesses des roues → vitesses du robot)
    // La fonction met à jour directement les vitesses dans la structure
    MecanumOdometry_updateKinematics(odometry, wheelVelocities);
    
    // Intégration pour calculer la nouvelle position
    // Transformation des vitesses du repère robot vers le repère monde
    float cos_theta = cos(odometry->pose.theta);
    float sin_theta = sin(odometry->pose.theta);
    
    float vx_world = odometry->velocity.vx * cos_theta - odometry->velocity.vy * sin_theta;
    float vy_world = odometry->velocity.vx * sin_theta + odometry->velocity.vy * cos_theta;
    
    // Intégration simple (Euler) - pourrait être améliorée avec Runge-Kutta
    odometry->pose.x += vx_world * deltaTime;
    odometry->pose.y += vy_world * deltaTime;
    odometry->pose.theta += odometry->velocity.omega * deltaTime;
    
    // Normalisation de l'angle
    odometry->pose.theta = MecanumOdometry_normalizeAngle(odometry->pose.theta);
}

/**
 * @brief Calcule la cinématique directe pour roues mécanum
 * @param odometry Pointeur vers la structure d'odométrie (les vitesses y seront stockées)
 * @param wheelVel Tableau des vitesses des 4 roues en m/s [FL, FR, RL, RR]
 * @return Aucun
 * @note Utilise les formules pour configuration mécanum standard (roues à 45°)
 * @note Met à jour directement odometry->velocity.vx, vy et omega
 */
void MecanumOdometry_updateKinematics(MecanumOdometry_t* odometry, float wheelVel[4])
{
    float lx = odometry->wheelBase / 2.0f;   // Demi-distance avant/arrière
    float ly = odometry->trackWidth / 2.0f;  // Demi-distance gauche/droite      // Somme des demi-distances
    
    // Facteurs d'échelle précalculés (plus efficace)
    float scale_linear = odometry->wheelRadius / 4.0f;           // R/4 pour vx et vy
    float scale_angular = odometry->wheelRadius / (4.0f * (lx + ly));  // R/(4*(lx+ly)) pour ω
    
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
    // Mise à jour directe des vitesses dans la structure
    odometry->velocity.vx = scale_linear * (wheelVel[0] + wheelVel[1] + wheelVel[2] + wheelVel[3]);
    odometry->velocity.vy = scale_linear * (-wheelVel[0] + wheelVel[1] + wheelVel[2] - wheelVel[3]);
    odometry->velocity.omega = scale_angular * (-wheelVel[0] + wheelVel[1] - wheelVel[2] + wheelVel[3]);
}

/**
 * @brief Calcule la distance euclidienne parcourue depuis un point de départ
 * @param odometry Pointeur vers la structure d'odométrie (lecture seule)
 * @param startX Position X de départ en mètres
 * @param startY Position Y de départ en mètres
 * @return Distance parcourue en mètres
 * @note Utilise la formule sqrt((x-x0)² + (y-y0)²)
 */
float MecanumOdometry_getDistanceTraveled(const MecanumOdometry_t* odometry, float startX, float startY)
{ 
    float dx = odometry->pose.x - startX;
    float dy = odometry->pose.y - startY;
    return sqrt(dx * dx + dy * dy);
}

/**
 * @brief Normalise un angle dans l'intervalle [-π, π]
 * @param angle Angle en radians à normaliser
 * @return Angle normalisé entre -π et π radians
 * @note Utilise la méthode de soustraction/addition de 2π
 */
float MecanumOdometry_normalizeAngle(float angle)
{
    // Si l'angle est supérieur à π, le ramener dans la plage
    if (angle > M_PI) {
        angle -= 2.0f * M_PI;
    }
    // Si l'angle est inférieur à -π, le ramener dans la plage
    else if (angle < -M_PI) {
        angle += 2.0f * M_PI;
    } 
    return angle;
}

/**
 * @brief Convertit des RPM en vitesse linéaire (m/s) pour toutes les roues
 * @param odometry Pointeur vers la structure d'odométrie (pour accéder au rayon des roues)
 * @param rpmArray Tableau des vitesses en RPM [FL, FR, RL, RR]
 * @param velocityArray Tableau de sortie des vitesses en m/s [FL, FR, RL, RR]
 * @return Aucun
 * @note Formule : vitesse(m/s) = RPM × (2π/60) × rayon_roue
 */
void MecanumOdometry_rpmArrayToMetersPerSecond(MecanumOdometry_t* odometry, float rpmArray[4], float velocityArray[4])
{
    // Facteur de conversion précalculé (plus efficace)
    float fRpmToMs = RPM_TO_RAD_S * odometry->wheelRadius;

    // Conversion vectorisée (4 roues d'un coup)
    for (int i = 0; i < 4; i++) {
        velocityArray[i] = rpmArray[i] * fRpmToMs;
    }
}

/**
 * @brief Affiche les informations d'odométrie sur le port série pour débogage
 * @param odometry Pointeur vers la structure d'odométrie (lecture seule)
 * @return Aucun
 * @note Affiche la position (X, Y, Theta) et les vitesses (Vx, Vy, Omega)
 * @note Format de sortie compatible avec Serial Plotter/Monitor
 */
void MecanumOdometry_debug(const MecanumOdometry_t* odometry)
{
    if (!odometry || !odometry->initialized) {
        Serial.println("Odometry not initialized");
        return;
    }
    
    Serial.print(">Odometry X en m:"); Serial.println(odometry->pose.x, 3);
    Serial.print(">Odometry Y en m:"); Serial.println(odometry->pose.y, 3);
    Serial.print(">Odometry Theta en rad:"); Serial.println(odometry->pose.theta, 3);
    Serial.print(">Velocity Vx en m/s:"); Serial.println(odometry->velocity.vx, 3);
    Serial.print(">Velocity Vy en m/s:"); Serial.println(odometry->velocity.vy, 3);
    Serial.print(">Velocity Omega en rad/s:"); Serial.println(odometry->velocity.omega, 3);
}