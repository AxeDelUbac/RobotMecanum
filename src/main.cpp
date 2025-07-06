#include "main.h"

HallSensor hallSensor1;
rotaryEncoder rotaryEncoder1;

movementController movementController1;
globalSpeed globalSpeed1;
MotionManager motionManager;
rotaryEncoder rotaryEncoderFrontLeft;

motorGearBox oleftFrontMotor(leftFrontMotorHigh, leftFrontMotorLow, leftFrontMotorPWM);
// ClosedLoopControl closedLoopControl1(&oleftFrontMotor, 2.0f, 1.0f, 0.01f);
ClosedLoopControl closedLoopControl1( 2.0f, 1.5f, 0.05f);


void task_create(void){
  Serial.println("Init FreeRTOS Task...");

  xTaskCreate(UartTask, "UartTask", 256, NULL, 1, NULL);
  xTaskCreate(MotorTask, "MotorTask", 256, NULL, 1, NULL);
  xTaskCreate(speedMesurementTask, "rotaryEncoderTask", 256, NULL, 1, NULL);
  xTaskCreate(displayInformationTask, "displayInformationTask", 256, NULL, 1, NULL);
  xTaskCreate(ImuProcessingTask, "ImuProcessingTask", 256, NULL, 1, NULL);
  xTaskCreate(PIDTask,"PIDTask", 256, NULL, 1, NULL);
}

extern volatile int iNombreTour[4];
extern volatile int iDirection[4];

volatile unsigned int nombreTours = 0;
volatile unsigned int nombreTours2 = 0;
volatile unsigned int nombreTours3 = 0;
volatile unsigned int nombreTours4 = 0;

volatile float PIDoutput = 0.0f;

HardwareSerial Serial2(PD6, PD5);

void setup() {

  GPIO_init();

  Serial.begin(115200);

  Serial2.begin(9600);

  delay(1000); // Laisse le temps au port série de s'initialiser

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

    // motionManager.GoUpBack();
    oleftFrontMotor.setMotorDirection(1, PIDoutput); // Avance à 100% de la vitesse

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

float vitesse = 0;
float vitessetopgauche = 0;
void speedMesurementTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {

    vitesse = globalSpeed1.getGlobalSpeedKmh(iNombreTour[0], iNombreTour[1], iNombreTour[2], iNombreTour[3]);
    vitessetopgauche = rotaryEncoderFrontLeft.getSpeedKmH(iNombreTour[1]);
    iNombreTour[0] = 0;
    iNombreTour[1] = 0;
    iNombreTour[2] = 0;
    iNombreTour[3] = 0;

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void displayInformationTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {

    Serial.print("Vitesse en km/h = ");
    Serial.println(vitesse),3;

      Serial.print("Vitesse top gauche km/h = ");
    Serial.println(vitessetopgauche),3;

    // Serial.print("Encodeur1= ");
    // Serial.println(iNombreTour[0]);
    // Serial.print("Direction1= ");
    // Serial.println(iDirection[0]);
    // Serial.print("Encodeur2= ");
    // Serial.println(iNombreTour[1]);
    // Serial.print("Direction2= ");
    // Serial.println(iDirection[1]);
    // Serial.print("Encodeur3= ");
    // Serial.println(iNombreTour[2]);
    // Serial.print("Direction3= ");
    // Serial.println(iDirection[2]);
    // Serial.print("Encodeur4= ");
    // Serial.println(iNombreTour[3]);
    // Serial.print("Direction4= ");
    // Serial.println(iDirection[3]);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void ImuProcessingTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void PIDTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {

    PIDoutput = closedLoopControl1.updatePIDControl(0.1f, vitesse);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
} 