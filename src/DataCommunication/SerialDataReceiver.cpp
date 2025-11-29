#include "SerialDataReceiver.h"
#include <Arduino.h>
#include <string.h>

#ifndef SERIAL_START_BYTE
#define SERIAL_START_BYTE 0xAA
#endif

MonitoringPacket_t s_lastMonitoringPacket;

// Lecture octet-par-octet : machine à états
typedef enum {
    UART_RECEIVE_WAIT_START_STATE = 0,
    UART_RECEIVE_READ_LENGTH_STATE,
    UART_RECEIVE_READ_TYPE_STATE,
    UART_RECEIVE_READ_DATA_STATE,
    UART_RECEIVE_READ_CHECKSUM_STATE
} RecvState_t;

static RecvState_t s_state = UART_RECEIVE_WAIT_START_STATE;
static uint8_t s_expectedLength = 0;
static uint8_t s_packetType = 0;
static uint8_t s_dataBuf[256];
static uint8_t s_dataIdx = 0;

// Statistiques pour diagnostic
static uint32_t s_statsFramesComplete = 0;
static uint32_t s_statsFramesMonitoring = 0; 
static uint32_t s_statsFramesMotor = 0;
static uint32_t s_statsFramesUnknown = 0;
static uint32_t s_statsBytesIgnored = 0;

void SerialDataReceiver_init(HardwareSerial* serialPort)
{
    serialPort = serialPort;
    s_state = UART_RECEIVE_WAIT_START_STATE;
    s_expectedLength = 0;
    s_packetType = 0;
    s_dataIdx = 0;
}

// A appeler régulièrement : lit tous les octets disponibles et les traite
void SerialDataReceiver_process(HardwareSerial* serialPort, MonitoringPacket_t* outMonitoringPacket)
{
    while (serialPort->available()) {
        int v = serialPort->read();
        if (v < 0) break;
        uint8_t b = (uint8_t)v;

        switch(s_state) 
        {
            case UART_RECEIVE_WAIT_START_STATE:
                if (b == SERIAL_START_BYTE) {
                    s_state = UART_RECEIVE_READ_LENGTH_STATE;
                } else {
                    // Afficher occasionnellement les octets ignorés pour debug
                    static uint8_t ignoreCount = 0;
                    s_statsBytesIgnored++;
                    if (++ignoreCount >= 50) { // Afficher 1 fois sur 50
                        Serial.print("Ignoré: 0x");
                        Serial.println(b, HEX);
                        ignoreCount = 0;
                    }
                }
            break;
            case UART_RECEIVE_READ_LENGTH_STATE:
                s_expectedLength = b; 
                Serial.print("Longueur attendue: ");
                Serial.println(s_expectedLength);
                s_state = UART_RECEIVE_READ_TYPE_STATE;
            break;
            case UART_RECEIVE_READ_TYPE_STATE:
                s_packetType = b;
                Serial.print("Type paquet: 0x");
                Serial.println(s_packetType, HEX);
                s_dataIdx = 0;
                if (s_expectedLength == 0) {
                    s_state = UART_RECEIVE_READ_CHECKSUM_STATE;
                } else {
                    s_state = UART_RECEIVE_READ_DATA_STATE;
                }
            break;
            case UART_RECEIVE_READ_DATA_STATE:
                if (s_dataIdx < sizeof(s_dataBuf)) {
                    s_dataBuf[s_dataIdx++] = b;
                } else {
                    // overflow -> reset
                    s_state = UART_RECEIVE_WAIT_START_STATE;
                    s_dataIdx = 0;
                    return;
                }
                if (s_dataIdx >= s_expectedLength) {
                    s_state = UART_RECEIVE_READ_CHECKSUM_STATE;
                }
            break;
            case UART_RECEIVE_READ_CHECKSUM_STATE:
            {
                // On lit le checksum mais on ne le vérifie pas (implémentation future)
                uint8_t receivedCs = b;
                (void)receivedCs;
                // Si debug activé, afficher la trame complète reçue
                Serial.print(" len="); Serial.print(s_expectedLength);
                Serial.print(" type="); Serial.print(s_packetType, HEX);
                Serial.print(" payload:");
                for (uint8_t i = 0; i < s_expectedLength; ++i) {
                    Serial.print(' ');
                    if (s_dataBuf[i] < 16) Serial.print('0');
                    Serial.print(s_dataBuf[i], HEX);
                }
                Serial.println();

                // Traitement selon le type de paquet
                s_statsFramesComplete++;
                if (s_packetType == PACKET_TYPE_MONITORING) {
                    s_statsFramesMonitoring++;
                    // Format attendu: [dirX, dirY, speedRatio] (3 octets)
                    if (s_expectedLength >= 3) {
                        outMonitoringPacket->dirX = s_dataBuf[0];
                        outMonitoringPacket->dirY = s_dataBuf[1];
                        outMonitoringPacket->omega = s_dataBuf[2];
                        outMonitoringPacket->speedRatio = s_dataBuf[3];

                        Serial.print("MonPkt: dirX="); Serial.print(outMonitoringPacket->dirX);
                        Serial.print(" dirY="); Serial.print(outMonitoringPacket->dirY);
                        Serial.print(" omega="); Serial.print(outMonitoringPacket->omega);
                        Serial.print(" speedRatio="); Serial.println(outMonitoringPacket->speedRatio);
                    }
                } else if (s_packetType == PACKET_TYPE_COMPLETE_DATA) {
                    s_statsFramesMotor++;
                    Serial.println("Reçu paquet de données moteur complet (non traité pour monitoring)");
                } else {
                    s_statsFramesUnknown++;
                    Serial.print("Type de paquet inconnu: 0x");
                    Serial.println(s_packetType, HEX);
                }
                // reset machine
                s_state = UART_RECEIVE_WAIT_START_STATE;
                s_dataIdx = 0;
                s_expectedLength = 0;
                s_packetType = 0;
            }
            break;
            default:
                s_state = UART_RECEIVE_WAIT_START_STATE;
            break;
        }
    }
}

void SerialDataReceiver_debug(void)
{
    Serial.print("État RX: ");
    switch (s_state) {
        case UART_RECEIVE_WAIT_START_STATE: Serial.println("WAIT_START"); break;
        case UART_RECEIVE_READ_LENGTH_STATE: Serial.println("READ_LENGTH"); break;
        case UART_RECEIVE_READ_TYPE_STATE: Serial.println("READ_TYPE"); break;
        case UART_RECEIVE_READ_DATA_STATE: Serial.println("READ_DATA"); break;
        case UART_RECEIVE_READ_CHECKSUM_STATE: Serial.println("READ_CHECKSUM"); break;
        default: Serial.println("UNKNOWN"); break;
    }
}

void SerialDataReceiver_printStats(void)
{
    Serial.println("=== STATS RÉCEPTEUR ===");
    Serial.print("Trames complètes: "); Serial.println(s_statsFramesComplete);
    Serial.print("- Monitoring: "); Serial.println(s_statsFramesMonitoring);
    Serial.print("- Moteur: "); Serial.println(s_statsFramesMotor);
    Serial.print("- Inconnues: "); Serial.println(s_statsFramesUnknown);
    Serial.print("Octets ignorés: "); Serial.println(s_statsBytesIgnored);
    Serial.println("========================");
}