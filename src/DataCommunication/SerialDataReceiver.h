#ifndef SERIAL_DATA_RECEIVER_H
#define SERIAL_DATA_RECEIVER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "SerialDataStructure.h"

// API
void SerialDataReceiver_init(HardwareSerial* serialPort);
void SerialDataReceiver_process(HardwareSerial* serialPort, MonitoringPacket_t* outMonitoringPacket);
void SerialDataReceiver_debug(void);
void SerialDataReceiver_printStats(void);

#endif // SERIAL_DATA_RECEIVER_H
