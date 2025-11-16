#include "bluetoothReception.h"

HardwareSerial BluetoothSerial(PC10, PC11);

void BluetoothReception_init(void) {
    BluetoothSerial.begin(9600);
}

char BluetoothReception_retreiveData(void) {
    if (BluetoothSerial.available()) {
        char cBluetoothCommand = BluetoothSerial.read();
        Serial.print("Reçu via BT : ");
        Serial.println(cBluetoothCommand);
        return cBluetoothCommand; // Return the received character
    }
    return -1; // Indicate no data was received
}

void BluetoothReception_debug(void) {
    BluetoothSerial.println("blue");
    Serial.println("Test Bluetooth Reception");
}