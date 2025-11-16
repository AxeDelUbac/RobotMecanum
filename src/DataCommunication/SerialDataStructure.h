#include <Arduino.h>
// Structure pour les données des moteurs à transmettre
typedef struct {
    float wheelSpeedRPM[4];     // Vitesses des 4 roues en RPM [FL, FR, RL, RR]
    float motorPower[4];        // Puissance des 4 moteurs en % [-100, 100]
} MotorDataPacket_t;

// Structure pour les données d'odométrie à transmettre
typedef struct {
    float positionX;            // Position X en m
    float positionY;            // Position Y en m
    float orientation;          // Orientation en rad
    float velocityX;            // Vitesse X en m/s
    float velocityY;            // Vitesse Y en m/s
    float angularVelocity;      // Vitesse angulaire en rad/s
} OdometryDataPacket_t;

// Structure pour les données de la centrale inertielle (IMU)
typedef struct {
    float accel[3];             // Accélération(m/s²) sur Axe X, Y, Z     
    float gyro[3];              // Vitesse angulaire(rad/s) sur Axe X, Y, Z
    float mag[3];               // Champ magnétique(µT) sur Axe X, Y, Z
    float roll;                 // Angle de roulis
    float pitch;                // Angle de tangage
    float yaw;                  // Angle de lacet
} ImuDataPacket_t;

// Structure unifiée pour toutes les données du robot (PAQUET COMPLET)
typedef struct {
    MotorDataPacket_t motorData;      // Données des moteurs
    OdometryDataPacket_t odometryData; // Données d'odométrie
    ImuDataPacket_t imuData;          // Données IMU
} DataPacket_t;

// Structure d'en-tête de paquet
typedef struct {
    uint8_t startByte;          // Octet de début (0xAA)
    uint8_t dataLength;        // Taille des données
    uint8_t packetType;         // Type de paquet
    DataPacket_t data; // Données complètes du robot
    uint8_t checksum;           // Checksum simple
} RobotDataPacket_t;

// Énumération des types de paquets
typedef enum {
    PACKET_TYPE_COMPLETE_DATA = 0x04,    // Type pour paquet complet
} PacketType_t;