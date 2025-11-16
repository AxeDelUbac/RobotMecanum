#ifndef SERIAL_DATA_TRANSMITTER_H
#define SERIAL_DATA_TRANSMITTER_H

#include <Arduino.h>
#include <HardwareSerial.h>

#include "SerialDataStructure.h"

// Configuration de la communication
typedef struct {
    HardwareSerial* serialPort; // Port série à utiliser
    uint32_t baudRate;          // Vitesse de transmission     // Activer la vérification checksum
} SerialCommConfig_t;

// Fonctions d'initialisation
void SerialDataTransmitter_init(SerialCommConfig_t* config, HardwareSerial* serialPort, uint32_t baudRate);

// Fonctions de transmission
bool SerialDataTransmitter_sendCompleteData(SerialCommConfig_t* config,MotorDataPacket_t* motorData);

// Fonctions utilitaires
uint8_t SerialDataTransmitter_calculateChecksum(const uint8_t* data, size_t length);
void SerialDataTransmitter_debug(void);

#endif // SERIAL_DATA_TRANSMITTER_H