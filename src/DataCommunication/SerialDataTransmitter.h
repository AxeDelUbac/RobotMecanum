#ifndef SERIAL_DATA_TRANSMITTER_H
#define SERIAL_DATA_TRANSMITTER_H

#include <Arduino.h>
#include <HardwareSerial.h>

// Structure pour les données des moteurs à transmettre
typedef struct {
    float wheelSpeedRPM[4];     // Vitesses des 4 roues en RPM [FL, FR, RL, RR]
    float motorPower[4];        // Puissance des 4 moteurs en % [-100, 100]
    // uint32_t timestamp;      // Optionnel : Timestamp en ms (pour sync temporelle)
} MotorDataPacket_t;

// Structure pour les données d'odométrie à transmettre
typedef struct {
    float positionX;            // Position X en m
    float positionY;            // Position Y en m
    float orientation;          // Orientation en rad
    float velocityX;            // Vitesse X en m/s
    float velocityY;            // Vitesse Y en m/s
    float angularVelocity;      // Vitesse angulaire en rad/s
    // uint32_t timestamp;      // Optionnel : Timestamp en ms (pour sync temporelle)
} OdometryDataPacket_t;

// Structure pour les données de la centrale inertielle (IMU)
typedef struct {
    float accel[3];             // Accélération(m/s²) sur Axe X, Y, Z     
    float gyro[3];              // Vitesse angulaire(rad/s) sur Axe X, Y, Z
    float mag[3];               // Champ magnétique(µT) sur Axe X, Y, Z
    float roll;                 // Angle de roulis
    float pitch;                // Angle de tangage
    float yaw;                  // Angle de lacet
    // uint32_t timestamp;      // Optionnel : Timestamp en ms
} ImuDataPacket_t;

// Structure unifiée pour toutes les données du robot (PAQUET COMPLET)
typedef struct {
    MotorDataPacket_t motorData;      // Données des moteurs
    OdometryDataPacket_t odometryData; // Données d'odométrie
    ImuDataPacket_t imuData;          // Données IMU
    uint32_t timestamp;               // Timestamp en ms pour tout le paquet
    uint16_t packetId;               // ID du paquet complet
} RobotCompleteDataPacket_t;

// Structure d'en-tête de paquet
typedef struct {
    uint8_t startByte;          // Octet de début (0xAA)
    uint8_t packetType;         // Type de paquet
    uint16_t dataLength;        // Taille des données
    uint8_t checksum;           // Checksum simple
} PacketHeader_t;

// Énumération des types de paquets
typedef enum {
    PACKET_TYPE_COMPLETE_DATA = 0x04,    // Type pour paquet complet
} PacketType_t;

// Configuration de la communication
typedef struct {
    HardwareSerial* serialPort; // Port série à utiliser
    uint32_t baudRate;          // Vitesse de transmission
    uint16_t transmissionRate;  // Fréquence d'envoi en Hz
    bool enableChecksum;        // Activer la vérification checksum
} SerialCommConfig_t;

// Structure principale du transmetteur
typedef struct {
    SerialCommConfig_t config;
    uint16_t nextPacketId;
    unsigned long lastTransmissionTime;
    bool initialized;
} SerialDataTransmitter_t;

// Fonctions d'initialisation
void SerialDataTransmitter_init(SerialDataTransmitter_t* transmitter, HardwareSerial* serialPort, uint32_t baudRate, uint16_t transmissionRate, bool enableChecksum);

// Fonctions de transmission
bool SerialDataTransmitter_sendCompleteData(SerialDataTransmitter_t* transmitter, const RobotCompleteDataPacket_t* completeData);

// Fonctions utilitaires
uint8_t SerialDataTransmitter_calculateChecksum(const uint8_t* data, size_t length);
void SerialDataTransmitter_debug(const SerialDataTransmitter_t* transmitter);

// Macros utiles
#define SERIAL_START_BYTE 0xAA
#define SERIAL_DEFAULT_BAUD_RATE 115200
#define SERIAL_DEFAULT_TRANSMISSION_RATE 20  // 20Hz = 50ms

#endif // SERIAL_DATA_TRANSMITTER_H