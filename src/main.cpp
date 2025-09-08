#include "main.h"

movementController oMovementController;
globalSpeed oGlobalSpeed;

rotaryEncoder rotaryEncoderFrontLeft;
rotaryEncoder rotaryEncoderFrontRight;
rotaryEncoder rotaryEncoderRearLeft;
rotaryEncoder rotaryEncoderRearRight;

GlobalControl oGlobalControl;
PositionOrientation oPositionOrientation;


void task_create(void){
  Serial.println("Init FreeRTOS Task...");

  xTaskCreate(UartTask, "UartTask", 256, NULL, 1, NULL);
  xTaskCreate(MotorTask, "MotorTask", 256, NULL, 1, NULL);
  xTaskCreate(speedMesurementTask, "rotaryEncoderTask", 256, NULL, 1, NULL);
  xTaskCreate(displayInformationTask, "displayInformationTask", 256, NULL, 1, NULL);
  xTaskCreate(ImuProcessingTask, "ImuProcessingTask", 256, NULL, 1, NULL);
  xTaskCreate(PIDTask,"PIDTask", 256, NULL, 1, NULL);
  xTaskCreate(IMUTask,"IMUTask", 256, NULL, 1, NULL);
}

extern volatile int iNombreTour[4];
extern volatile int iDirection[4];

volatile unsigned int nombreTours = 0;
volatile unsigned int nombreTours2 = 0;
volatile unsigned int nombreTours3 = 0;
volatile unsigned int nombreTours4 = 0;

float vitesseEncoder[4] = {0, 0, 0, 0};

float PIDoutput[4] = {0.0f, 0.0f, 0.0f, 0.0f};

HardwareSerial Serial2(PD6, PD5);

void setup() {

  GPIO_init();


  Serial.begin(115200);

  Serial2.begin(9600);

  createIT();

  task_create();
  // Démarrer le scheduler FreeRTOS
  vTaskStartScheduler();

}

void loop() 
{

}

void UartTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    Serial2.println("Hello from UartTask");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void MotorTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    oMovementController.setMotorSpeedInPWM(PIDoutput);
    oMovementController.movementFront();

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

float vitesse = 0;
const float fSpeedMesurementPeriodMs = 100;

void speedMesurementTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {

    vitesse = oGlobalSpeed.getGlobalSpeedKmh(iNombreTour[0], iNombreTour[1], iNombreTour[2], iNombreTour[3],fSpeedMesurementPeriodMs);

    vitesseEncoder[0] = rotaryEncoderFrontLeft.getSpeedRpm(iNombreTour[1], fSpeedMesurementPeriodMs);
    vitesseEncoder[1] = rotaryEncoderFrontRight.getSpeedRpm(iNombreTour[2], fSpeedMesurementPeriodMs);
    vitesseEncoder[2] = rotaryEncoderRearLeft.getSpeedRpm(iNombreTour[3], fSpeedMesurementPeriodMs);
    vitesseEncoder[3] = rotaryEncoderRearRight.getSpeedRpm(iNombreTour[0], fSpeedMesurementPeriodMs);

    iNombreTour[0] = 0;
    iNombreTour[1] = 0;
    iNombreTour[2] = 0;
    iNombreTour[3] = 0;

    vTaskDelay(pdMS_TO_TICKS(fSpeedMesurementPeriodMs));
  }
}

float Accel[3];
float Gyro[3];
float roll, pitch, yaw;

void displayInformationTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {

    oGlobalControl.SerialDebug();
    oGlobalSpeed.serialDebug();

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void ImuProcessingTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
float fOutputKmh[4];

void PIDTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {

    oGlobalControl.UpdateSetpoint(300, vitesseEncoder, PIDoutput);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
} 

void IMUTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {

    oPositionOrientation.begin();
    oPositionOrientation.update(0.1f);
    oPositionOrientation.getEulerAngles(roll, pitch, yaw); // Mettre à jour la position toutes les 100 ms

    vTaskDelay(pdMS_TO_TICKS(100));
  }
} 