#include "main.h"

movementController oMovementController;
globalSpeed oGlobalSpeed;

rotaryEncoder rotaryEncoderFrontLeft;
rotaryEncoder rotaryEncoderFrontRight;
rotaryEncoder rotaryEncoderRearLeft;
rotaryEncoder rotaryEncoderRearRight;

GlobalControl tGlobalControl;


void task_create(void){
  Serial.println("Init FreeRTOS Task...");

  xTaskCreate(MotorRegulationTask, "MotorRegulationTask", 256, NULL, 1, NULL);
  xTaskCreate(speedMesurementTask, "rotaryEncoderTask", 256, NULL, 1, NULL);
  xTaskCreate(displayInformationTask, "displayInformationTask", 256, NULL, 1, NULL);
  xTaskCreate(commandProcessingTask, "commandProcessingTask", 256, NULL, 1, NULL);
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

float fsetpointRpm = 100;

HardwareSerial Serial2(PD6, PD5);

void setup() {

  GPIO_init();

  GlobalControl_init(&tGlobalControl);
  CommandProcessing_init();

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

void MotorRegulationTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    GlobalControl_UpdateSetpoint(&tGlobalControl, fsetpointRpm, vitesseEncoder, PIDoutput);
    oMovementController.setMotorSpeedInPWM(PIDoutput);
    oMovementController.movementFront();

    vTaskDelay(pdMS_TO_TICKS(100));
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

void displayInformationTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {

    // GlobalControl_SerialDebug(&tGlobalControl);
    // oGlobalSpeed.serialDebug();

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


void commandProcessingTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    fsetpointRpm = CommandProcessing_modifySetpointInRpm();
    Serial.print("Setpoint RPM: ");
    Serial.println(fsetpointRpm);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void IMUTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {

    PositionOrientation_begin();
    PositionOrientation_update(0.1f);
    // PositionOrientation_getEulerAngles(roll, pitch, yaw); // Mettre à jour la position toutes les 100 ms

    vTaskDelay(pdMS_TO_TICKS(100));
  }
} 