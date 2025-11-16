#ifndef BLUETOOTH_RECEPTION_H
#define BLUETOOTH_RECEPTION_H

#include <Arduino.h>

void BluetoothReception_init(void);
char BluetoothReception_retreiveData(void);
void BluetoothReception_debug(void) ;

#endif // BLUETOOTH_RECEPTION_H