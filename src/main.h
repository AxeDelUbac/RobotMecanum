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

void UartTask(void *pvParameters);
void MotorTask(void *pvParameters);
void speedMesurementTask(void *pvParameters);
void displayInformationTask(void *pvParameters);
void ImuProcessingTask(void *pvParameters);
void PIDTask(void *pvParameters);
void IMUTask(void *pvParameters);

void task_create(void);