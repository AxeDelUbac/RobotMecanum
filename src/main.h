#include <Arduino.h>
#include <STM32FreeRTOS.h>

#include "GPIO.h"
#include "IT.h"

#include "motorController/movementController.h"
#include "speedMesurement/GlobalSpeed.h"
#include "speedControlSystem/GlobalControl.h"

// #include "positionManagement/Imu.h"
#include "positionManagement/Imu.h"
#include "positionManagement/PositionOrientation.h"

#include "navigation/forwardKinematics.h"
#include "navigation/mecanumOdometrie.h"

#include "DataCommunication/SerialDataTransmitter.h"
#include "DataCommunication/SerialDataReceiver.h"

void displayInformationTask(void *pvParameters);
void MotorRegulationTask(void *pvParameters);
void speedMesurementTask(void *pvParameters);
void IMUTask(void *pvParameters);
void DataTransmissionTask(void *pvParameters);
void UartTransmitTask(void *pvParameters);
void UartReceiveTask(void *pvParameters);

void task_create(void);