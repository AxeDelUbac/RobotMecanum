#include <Arduino.h>
#include <STM32FreeRTOS.h>

#include "GPIO.h"
#include "IT.h"

#include "motorController/movementController.h"
#include "speedMesurement/rotaryEncoder.h"
#include "speedMesurement/globalSpeed.h"

// #include "speedControlSystem/closedLoopControl.h"
#include "speedControlSystem/GlobalControl.h"

// #include "positionManagement/Imu.h"
#include "positionManagement/PositionOrientation.h"

#include "commandProcessing/commandProcessing.h"

void MotorRegulationTask(void *pvParameters);
void speedMesurementTask(void *pvParameters);
void displayInformationTask(void *pvParameters);
void commandProcessingTask(void *pvParameters);
void IMUTask(void *pvParameters);

void task_create(void);