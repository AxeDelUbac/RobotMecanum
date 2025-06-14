#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "main.h"

#include "motorController/motionManager.h"
#include "motorController/movementController.h"
#include "speedMesurement/rotaryEncoder.h"
#include "speedMesurement/globalSpeed.h"

void UartTask(void *pvParameters);
void MotorTask(void *pvParameters);
void speedMesurementTask(void *pvParameters);
void displayInformationTask(void *pvParameters);

void task_create(void);

void task_create(void){
  Serial.println("Init FreeRTOS Task...");
  // Créer deux tâches
  xTaskCreate(UartTask, "UartTask", 256, NULL, 1, NULL);
  xTaskCreate(MotorTask, "MotorTask", 256, NULL, 1, NULL);
  xTaskCreate(speedMesurementTask, "rotaryEncoderTask", 256, NULL, 1, NULL);
  xTaskCreate(displayInformationTask, "displayInformationTask", 256, NULL, 1, NULL);

  Serial.println("Done");
}

volatile unsigned int nombreTours = 0;
volatile unsigned int nombreTours2 = 0;
volatile unsigned int nombreTours3 = 0;
volatile unsigned int nombreTours4 = 0;

HallSensor hallSensor1;
rotaryEncoder rotaryEncoder1;
movementController movementController1;
globalSpeed globalSpeed1;
MotionManager motionManager;

HardwareSerial Serial2(PD6, PD5);

void compterTour() {
  nombreTours++;
}

void compterTour2() {
  nombreTours2++;
}

// void compterTour3() {
//   nombreTours3++;
// }

// void compterTour4() {
//   nombreTours4++;
// }

void setup() {
  // put your setup code here, to run once:
  pinMode(rightBackMotorHigh, OUTPUT);
  pinMode(rightBackMotorLow, OUTPUT);
  pinMode(rightBackMotorPWM, OUTPUT);

  pinMode(leftBackMotorHigh, OUTPUT);
  pinMode(leftBackMotorLow, OUTPUT);
  pinMode(leftBackMotorPWM, OUTPUT);

  pinMode(rightFrontMotorHigh, OUTPUT);
  pinMode(rightFrontMotorLow, OUTPUT);
  pinMode(rightFrontMotorPWM, OUTPUT);
    pinMode(rightFrontFirstHallSensor, INPUT);
  pinMode(rightFrontSecondHallSensor, INPUT);

  pinMode(leftFrontMotorHigh, OUTPUT);
  pinMode(leftFrontMotorLow, OUTPUT);
  pinMode(leftFrontMotorPWM, OUTPUT);
  pinMode(leftFrontFirstHallSensor, INPUT);
  pinMode(leftFrontSecondHallSensor, INPUT);

  Serial.begin(115200);


  Serial2.begin(9600);

  delay(1000); // Laisse le temps au port série de s'initialiser

  attachInterrupt(digitalPinToInterrupt(rightFrontFirstHallSensor), compterTour, FALLING); // Détection du front descendant
  attachInterrupt(digitalPinToInterrupt(rightFrontFirstHallSensor), compterTour2, FALLING); // Détection du front descendant
  // attachInterrupt(digitalPinToInterrupt(leftBackFirstHallSensor), compterTour3, FALLING); // Détection du front descendant
  // attachInterrupt(digitalPinToInterrupt(leftBackSecondHallSensor), compterTour4, FALLING); // Détection du front descendant

  task_create();
  // Démarrer le scheduler FreeRTOS
  vTaskStartScheduler();

}

void loop() {
  // put your main code here, to run repeatedly:
  /*Serial.println("Ahah");
  delay(100);*/
}

void UartTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    Serial2.println("Hello from UartTask");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

  // ---------- Tâche 2 ----------
  void MotorTask(void *pvParameters) {
    (void) pvParameters;
    while (1) {

      motionManager.GoUpBack();

      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }

  float vitesse = 0;
  void speedMesurementTask(void *pvParameters) {
    (void) pvParameters;
    while (1) {

      vitesse = rotaryEncoder1.getMeanSpeedKmh(nombreTours, nombreTours2);
      // vitesse = globalSpeed1.getGlobalSpeedKmh(nombreTours, nombreTours2, nombreTours3, nombreTours4);
      nombreTours = 0;
      nombreTours2 = 0;
      // nombreTours3 = 0;
      // nombreTours4 = 0;


      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  void displayInformationTask(void *pvParameters) {
    (void) pvParameters;
    while (1) {

      /*Serial.print("Encodeur1= ");
      Serial.println(nombreTours);
      Serial.print("Encodeur2= ");
      Serial.println(nombreTours2);*/
      Serial.print("Vitesse en km/h = ");
      Serial.println(vitesse),3;

      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }