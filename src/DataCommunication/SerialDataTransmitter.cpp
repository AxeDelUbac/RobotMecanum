#include "SerialDataTransmitter.h"

/**
 * @brief Initialise le système de transmission série avec configuration complète
 * @param transmitter Pointeur vers la structure du transmetteur
 * @param serialPort Pointeur vers le port série à utiliser (Serial2, Serial3, etc.)
 * @param baudRate Vitesse de transmission en bauds
 * @param transmissionRate Fréquence d'envoi en Hz
 * @param enableChecksum Activer la vérification par checksum
 * @return Aucun
 */
void SerialDataTransmitter_init(SerialDataTransmitter_t* transmitter, HardwareSerial* serialPort, uint32_t baudRate, uint16_t transmissionRate, bool enableChecksum)
{
    // Configuration du port série
    transmitter->config.serialPort = serialPort;
    transmitter->config.baudRate = baudRate;
    transmitter->config.transmissionRate = transmissionRate;
    transmitter->config.enableChecksum = enableChecksum;
    
    // Initialisation des variables
    transmitter->nextPacketId = 1;
    transmitter->lastTransmissionTime = 0;
    transmitter->initialized = false;
    
    // Démarrage du port série
    serialPort->begin(baudRate);
    transmitter->initialized = true;
    
    Serial.println("Serial Data Transmitter initialized");
    Serial.print("UART Port configured at: "); Serial.print(baudRate); Serial.println(" baud");
    Serial.print("Transmission rate set to: "); Serial.print(transmissionRate); Serial.println(" Hz");
    Serial.print("Checksum: "); Serial.println(enableChecksum ? "ENABLED" : "DISABLED");
}

/**
 * @brief Transmet toutes les données du robot (moteurs + odométrie + IMU) en un seul paquet
 * @param transmitter Pointeur vers la structure du transmetteur
 * @param completeData Pointeur vers les données complètes du robot
 * @return true si la transmission a réussi, false sinon
 * @note Plus efficace que d'envoyer 3 paquets séparés
 */
bool SerialDataTransmitter_sendCompleteData(SerialDataTransmitter_t* transmitter, const RobotCompleteDataPacket_t* completeData)
{
    if (!transmitter || !transmitter->initialized || !completeData) return false;
    
    HardwareSerial* serial = transmitter->config.serialPort;
    
    // Création de l'en-tête
    PacketHeader_t header;
    header.startByte = SERIAL_START_BYTE;
    header.dataLength = sizeof(RobotCompleteDataPacket_t);
    
    // Calcul du checksum si activé
    if (transmitter->config.enableChecksum == true) 
    {
        header.checksum = SerialDataTransmitter_calculateChecksum((const uint8_t*)completeData, sizeof(RobotCompleteDataPacket_t));
    } 
    else 
    {
        header.checksum = 0;
    }
    
    // Transmission de l'en-tête
    serial->write((uint8_t*)&header, sizeof(PacketHeader_t));
    
    // Transmission des données complètes
    serial->write((uint8_t*)completeData, sizeof(RobotCompleteDataPacket_t));
    
    // Mise à jour de l'ID de paquet
    transmitter->nextPacketId++;
    transmitter->lastTransmissionTime = millis();
    
    return true;
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
void SerialDataTransmitter_debug(const SerialDataTransmitter_t* transmitter)
{
    if (!transmitter) {
        Serial.println("Transmitter not initialized");
        return;
    }
    
    Serial.print("UART Transmitter Status: ");
    Serial.println(transmitter->initialized ? "OK" : "NOT_INIT");
    
    Serial.print("Baud Rate: "); Serial.println(transmitter->config.baudRate);
    Serial.print("Transmission Rate: "); Serial.print(transmitter->config.transmissionRate); Serial.println(" Hz");
    Serial.print("Next Packet ID: "); Serial.println(transmitter->nextPacketId);
    Serial.print("Checksum: "); Serial.println(transmitter->config.enableChecksum ? "ON" : "OFF");
}