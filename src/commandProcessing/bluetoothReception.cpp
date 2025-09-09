#include "bluetoothReception.h"

HardwareSerial Serial4(PC10, PC11);

void BluetoothReception_init(void) {
    Serial4.begin(9600);
}

char BluetoothReception_retreiveData(void) {
    if (Serial4.available()) {
        char cBluetoothCommand = Serial4.read();
        Serial.print("Reçu via BT : ");
        Serial.println(cBluetoothCommand);
        return cBluetoothCommand; // Return the received character
    }
    return -1; // Indicate no data was received
}