#include "SerialDataReceiver.h"
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

static volatile bool s_monitoringPacketAvailable = false;

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
                }
            break;
            case UART_RECEIVE_READ_LENGTH_STATE:
                s_expectedLength = b; // on suppose 1 octet length (0..255)
                s_state = UART_RECEIVE_READ_TYPE_STATE;
            break;
            case UART_RECEIVE_READ_TYPE_STATE:
                s_packetType = b;
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
                // On lit le checksum mais on ne le vérifie pas (implémentation future)
                uint8_t receivedCs = b;
                (void)receivedCs;
                // Si debug activé, afficher la trame complète reçue
                Serial.print(" len="); Serial.print(s_expectedLength);
                Serial.print(" payload:");
                for (uint8_t i = 0; i < s_expectedLength; ++i) {
                    Serial.print(' ');
                    if (s_dataBuf[i] < 16) Serial.print('0');
                    Serial.print(s_dataBuf[i], HEX);
                }
                Serial.println();

                // Format attendu: [dirX, dirY, speedRatio] (3 octets)
                if (s_expectedLength >= 3) {
                    outMonitoringPacket->dirX = s_dataBuf[0];
                    outMonitoringPacket->dirY = s_dataBuf[1];
                    outMonitoringPacket->speedRatio = s_dataBuf[2];

                    Serial.print("MonPkt: dirX="); Serial.print(outMonitoringPacket->dirX);
                    Serial.print(" dirY="); Serial.print(outMonitoringPacket->dirY);
                    Serial.print(" speedRatio="); Serial.println(outMonitoringPacket->speedRatio);
                }
                // reset machine
                s_state = UART_RECEIVE_WAIT_START_STATE;
                s_dataIdx = 0;
                s_expectedLength = 0;
                s_packetType = 0;
            break;
            default:
                s_state = UART_RECEIVE_WAIT_START_STATE;
            break;
        }
    }
}

void SerialDataReceiver_debug(void)
{

}