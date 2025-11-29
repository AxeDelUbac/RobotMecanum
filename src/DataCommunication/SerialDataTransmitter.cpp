#include "SerialDataTransmitter.h"
// debug serial
static HardwareSerial* s_debugSerial = NULL;
static bool s_debugEnabled = false;

void SerialDataTransmitter_setDebugSerial(HardwareSerial* debugSerial)
{
    s_debugSerial = debugSerial;
}

void SerialDataTransmitter_enableDebug(bool enable)
{
    s_debugEnabled = enable;
}
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
    
    // Construire un buffer propre avec header + payload + checksum
    uint8_t txBuffer[256];
    size_t bufIdx = 0;
    
    // Header
    txBuffer[bufIdx++] = packet.startByte;          // 0xAA
    txBuffer[bufIdx++] = packet.dataLength;         // taille payload
    txBuffer[bufIdx++] = packet.packetType;         // type
    
    // Payload (MotorDataPacket_t)
    memcpy(&txBuffer[bufIdx], motorData, sizeof(MotorDataPacket_t));
    bufIdx += sizeof(MotorDataPacket_t);
    
    // Checksum simple
    txBuffer[bufIdx++] = packet.checksum;
    
    // debug: afficher la trame envoyée si demandé
    if (s_debugEnabled && s_debugSerial) {
        s_debugSerial->print("TX:");
        for (size_t i = 0; i < bufIdx; ++i) {
            s_debugSerial->print(' ');
            if (txBuffer[i] < 16) s_debugSerial->print('0');
            s_debugSerial->print(txBuffer[i], HEX);
        }
        s_debugSerial->println();
    }

    // Transmission du buffer construit
    size_t bytesWritten = serial->write(txBuffer, bufIdx);

    return (bytesWritten == bufIdx);
}

/**
 * @brief Transmet un petit paquet de monitoring (dirX, dirY, speedRatio)
 * @param config Configuration de transmission
 * @param monitoringData Données de monitoring à envoyer
 * @return true si la transmission a réussi, false sinon
 */
bool SerialDataTransmitter_sendMonitoringPacket(SerialCommConfig_t* config, MonitoringPacket_t* monitoringData)
{
    HardwareSerial* serial = config->serialPort;
    
    // Construire un buffer propre avec header + payload + checksum
    uint8_t txBuffer[256];
    size_t bufIdx = 0;
    
    // Header
    txBuffer[bufIdx++] = 0xAA;                      // Start byte
    txBuffer[bufIdx++] = sizeof(MonitoringPacket_t); // 3 octets
    txBuffer[bufIdx++] = PACKET_TYPE_MONITORING;    // Type 0x10
    
    // Payload (MonitoringPacket_t)
    memcpy(&txBuffer[bufIdx], monitoringData, sizeof(MonitoringPacket_t));
    bufIdx += sizeof(MonitoringPacket_t);
    
    // Checksum simple
    txBuffer[bufIdx++] = 0xFC;
    
    // debug: afficher la trame envoyée
    Serial.print("TX:");
    for (size_t i = 0; i < bufIdx; ++i) {
        Serial.print(' ');
        if (txBuffer[i] < 16) Serial.print('0');
        Serial.print(txBuffer[i], HEX);
    }
    Serial.println();

    // Transmission du buffer construit
    size_t bytesWritten = serial->write(txBuffer, bufIdx);

    return (bytesWritten == bufIdx);
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