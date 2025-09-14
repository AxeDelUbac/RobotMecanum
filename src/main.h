#include <Arduino.h>
#include <STM32FreeRTOS.h>

#include "GPIO.h"
#include "IT.h"

#include "motorController/movementController.h"
#include "speedMesurement/GlobalSpeed.h"
#include "speedControlSystem/GlobalControl.h"

// #include "positionManagement/Imu.h"
#include "positionManagement/PositionOrientation.h"

#include "commandProcessing/commandProcessing.h"
#include "commandProcessing/bluetoothReception.h"

void displayInformationTask(void *pvParameters);
void MotorRegulationTask(void *pvParameters);
void speedMesurementTask(void *pvParameters);
void commandProcessingTask(void *pvParameters);
void IMUTask(void *pvParameters);

void task_create(void);