#include "SerialDataTransmitter.h"
/**
 * @brief Initialise le système de transmission série avec configuration complète
 * @param transmitter Pointeur vers la structure du transmetteur
 * @param serialPort Pointeur vers le port série à utiliser (Serial2, Serial3, etc.)
 * @param baudRate Vitesse de transmission en bauds
 * @return Aucun
 */
void SerialDataTransmitter_init(SerialCommConfig_t* config, HardwareSerial* serialPort, uint32_t baudRate)
{
    // Configuration du port série
    config->serialPort = serialPort;
    config->baudRate = baudRate;
    // Démarrage du port série
    serialPort->begin(config->baudRate);
}

/**
 * @brief Transmet toutes les données du robot (moteurs + odométrie + IMU) en un seul paquet
 * @param transmitter Pointeur vers la structure du transmetteur
 * @param completeData Pointeur vers les données complètes du robot
 * @return true si la transmission a réussi, false sinon
 * @note Plus efficace que d'envoyer 3 paquets séparés
 */
bool SerialDataTransmitter_sendCompleteData(SerialCommConfig_t* config, MotorDataPacket_t* motorData)
{
    HardwareSerial* serial = config->serialPort;
    
    // Création du paquet complet avec en-tête + données
    RobotDataPacket_t packet;
    packet.startByte = 0xAA;
    packet.dataLength = sizeof(MotorDataPacket_t);
    packet.packetType = PACKET_TYPE_COMPLETE_DATA;
    
    // Copie des données dans le paquet
    packet.data.motorData = *motorData;
    
    // Calcul du checksum sur les données uniquement
    packet.checksum = 0xfc;
    // packet.checksum = SerialDataTransmitter_calculateChecksum((uint8_t*)&packet.data, sizeof(DataPacket_t));
    
    // Transmission du paquet complet en binaire
    size_t bytesWritten = serial->write((uint8_t*)&packet, sizeof(MotorDataPacket_t));
    
    return (bytesWritten == sizeof(MotorDataPacket_t));
}

/**
 * @brief Calcule un checksum simple pour vérifier l'intégrité des données
 * @param data Pointeur vers les données
 * @param length Taille des données en octets
 * @return Checksum calculé
 */
uint8_t SerialDataTransmitter_calculateChecksum(const uint8_t* data, size_t length)
{
    if (!data) return 0;
    
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];  // XOR simple
    }
    return checksum;
}

/**
 * @brief Affiche les informations de debug du transmetteur
 * @param transmitter Pointeur vers la structure du transmetteur
 * @return Aucun
 */
void SerialDataTransmitter_debug(void)
{
}