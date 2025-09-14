#include "main.h"

rotaryEncoder rotaryEncoderFrontLeft;
rotaryEncoder rotaryEncoderFrontRight;
rotaryEncoder rotaryEncoderRearLeft;
rotaryEncoder rotaryEncoderRearRight;

GlobalControl tGlobalControl;
movementController_t tMovementController;


void task_create(void){
  Serial.println("Init FreeRTOS Task...");

  xTaskCreate(displayInformationTask, "displayInformationTask", 256, NULL, 1, NULL);
  xTaskCreate(MotorRegulationTask, "MotorRegulationTask", 256, NULL, 1, NULL);
  xTaskCreate(speedMesurementTask, "rotaryEncoderTask", 256, NULL, 1, NULL);
  xTaskCreate(commandProcessingTask, "commandProcessingTask", 256, NULL, 1, NULL);
  xTaskCreate(IMUTask,"IMUTask", 256, NULL, 1, NULL);
}

void System_componentsInit(void){
  GPIO_init();
  GlobalControl_init(&tGlobalControl);
  MovementController_init(&tMovementController);
  CommandProcessing_init();
  BluetoothReception_init();
  PositionOrientation_init();
  Imu_init();
}

extern volatile int iNombreTour[4];
extern volatile int iDirection[4];

volatile unsigned int nombreTours = 0;
volatile unsigned int nombreTours2 = 0;
volatile unsigned int nombreTours3 = 0;
volatile unsigned int nombreTours4 = 0;

float vitesseEncoder[4] = {0, 0, 0, 0};

float PIDoutput[4] = {0.0f, 0.0f, 0.0f, 0.0f};

float fsetpointRpm = 0;

HardwareSerial Serial2(PD6, PD5);

void setup() {

  Serial.begin(115200);
  Serial.println();
  Serial.println("Scan I2C1 démarré...");
  // Si besoin, préciser les pins : Wire1.begin(SDA_pin, SCL_pin);
  Wire.begin(); // initialise I2C1 avec les paramètres par défaut

  int nDevices = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Device trouvé à 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.print("  (");
      Serial.print(address);
      Serial.println(")");
      nDevices++;
    }
    // petit délai pour stabilité si nécessaire
    delay(2);
  }

  System_componentsInit();

  createIT();

  task_create();
  // Démarrer le scheduler FreeRTOS
  vTaskStartScheduler();

}

void loop() 
{

}

void displayInformationTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(200);
  while (1) {

    // GlobalControl_SerialDebug(&tGlobalControl);
    Imu_SerialDebug();

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void MotorRegulationTask(void *pvParameters) {
  while (1) {

  GlobalControl_UpdateSetpoint(&tGlobalControl, fsetpointRpm, vitesseEncoder, PIDoutput);

  MovementController_setMotorSpeedInPWM(&tMovementController, PIDoutput);
  MovementController_movementFront(&tMovementController);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

const float fSpeedMesurementPeriodMs = 100;

void speedMesurementTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(fSpeedMesurementPeriodMs);
  while (1) {

    vitesseEncoder[0] = rotaryEncoderFrontLeft.getSpeedRpm(iNombreTour[1], fSpeedMesurementPeriodMs);
    vitesseEncoder[1] = rotaryEncoderFrontRight.getSpeedRpm(iNombreTour[2], fSpeedMesurementPeriodMs);
    vitesseEncoder[2] = rotaryEncoderRearLeft.getSpeedRpm(iNombreTour[3], fSpeedMesurementPeriodMs);
    vitesseEncoder[3] = rotaryEncoderRearRight.getSpeedRpm(iNombreTour[0], fSpeedMesurementPeriodMs);

    iNombreTour[0] = 0;
    iNombreTour[1] = 0;
    iNombreTour[2] = 0;
    iNombreTour[3] = 0;

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void commandProcessingTask(void *pvParameters) {
  while (1) {

        fsetpointRpm = CommandProcessing_modifySetpointInRpm();

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void IMUTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(250);
  while (1) {

    Imu_getMagnetometer();
    Imu_updateOrientation(0.25f); 

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
} 