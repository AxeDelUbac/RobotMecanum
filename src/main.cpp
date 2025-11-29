#include "main.h"

GlobalControl tGlobalControl;
movementController_t tMovementController;
MecanumOdometry_t robotOdometry;

MonitoringPacket_t lastMonitoringPacket;

SerialCommConfig_t uartTransmitter;  // Type corrigé avec majuscule

// Variables pour la communication de données UART  
// HardwareSerial Uart4(PC10, PC11);  // UART4 : PC10=TX, PC11=RX (ordre corrigé)
HardwareSerial UartSerial(PC10, PC11);


void task_create(void){
  Serial.println("Init FreeRTOS Task...");

  xTaskCreate(displayInformationTask, "displayInformationTask", 256, NULL, 1, NULL);
  xTaskCreate(MotorRegulationTask, "MotorRegulationTask", 512, NULL, 2, NULL);
  xTaskCreate(speedMesurementTask, "rotaryEncoderTask", 256, NULL, 1, NULL);
  xTaskCreate(IMUTask,"IMUTask", 256, NULL, 1, NULL);
  xTaskCreate(UartTransmitTask,"UartTransmitTask", 256, NULL, 1, NULL);
  xTaskCreate(UartReceiveTask,"UartReceiveTask", 256, NULL, 1, NULL);
}

void System_componentsInit(void){
  GPIO_init();
  GlobalControl_init(&tGlobalControl);
  MovementController_init(&tMovementController);
  SerialDataTransmitter_init(&uartTransmitter, &UartSerial, 115200);  // Nom de variable corrigé
  SerialDataReceiver_init(&UartSerial);
  PositionOrientation_init();
  Imu_init();
  
  // Initialisation de l'odométrie
  float wheelRadius = wheelDiameter / 2.0f; // Utilise la constante du fichier encoderParameter.h
  MecanumOdometry_init(&robotOdometry, wheelRadius, 0.20f, 0.18f); // Ajustez les dimensions selon votre robot
  
  // Initialisation simple de UART4 pour transmission
  // Uart4.begin(9600);
  // Serial.println("UART4 initialized at 115200 baud");
}

float fWheelSpeed[4] = {0, 0, 0, 0};
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
  // (handler utilisateur retiré — le récepteur affiche la trame complète en debug par défaut)
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
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);
  while (1) {

    // GlobalSpeed_debug();
    // Imu_SerialDebug();
    // MecanumOdometry_debug(&robotOdometry);

    // Serial.print("Paquets reçus: ");
    // Serial.print(lastMonitoringPacket.dirX);
    // Serial.print(", ");
    // Serial.print(lastMonitoringPacket.dirY);
    // Serial.print(", ");
    // Serial.println(lastMonitoringPacket.speedRatio);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void MotorRegulationTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz pour asservissement + odométrie
  
  while (1) {
    float fNormalisedWheelSpeed[4] = {0, 0, 0, 0};

    // === CINEMATIQUE DIRECTE ===
    ForwardKinematics_computeWheelVelocities(lastMonitoringPacket.dirX, lastMonitoringPacket.dirY, lastMonitoringPacket.omega, fWheelSpeed);
    ForwardKinematics_normalizeMotorsSpeeds(fWheelSpeed, fNormalisedWheelSpeed, 100.0f);

    // === ASSERVISSEMENT MOTEUR ===
    // GlobalControl_UpdateSetpoint(&tGlobalControl, fsetpointRpm, fWheelSpeed, PIDoutput);
    MovementController_setMotorSpeedInPWM(&tMovementController, PIDoutput);

    MovementController_setMovement(&tMovementController, fNormalisedWheelSpeed);

    // === MISE À JOUR ODOMÉTRIE ===
    // Utilise les vitesses mesurées par les encodeurs pour calculer la position
    MecanumOdometry_updateWheelSpeeds(&robotOdometry, fWheelSpeed);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

const float fSpeedMesurementPeriodMs = 100;

void speedMesurementTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(fSpeedMesurementPeriodMs);
  while (1) {

    GlobalSpeed_getMeanSpeedInRPM(fWheelSpeed, fSpeedMesurementPeriodMs);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
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

/* 
 * UartTask - Tâche pour transmission des données via UART4
 */
void UartTransmitTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(7500); // 1 Hz (toutes les 1000ms)
  
  while (1) {
    
    // Préparation du paquet de données moteur
    MotorDataPacket_t motorData = {};
    for (int i = 0; i < 4; i++) {
      motorData.wheelSpeedRPM[i] = fWheelSpeed[i];
      motorData.motorPower[i] = PIDoutput[i];
    }

    // Transmission via notre transmetteur
    SerialDataTransmitter_sendCompleteData(&uartTransmitter, &motorData);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/* 
 * UartRxTask - Tâche pour réception des données via UART
 */
void UartReceiveTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50); // Vérification toutes les 50ms
  
  uint32_t statsCounter = 0;
  
  while (1) {
    SerialDataReceiver_process(&UartSerial, &lastMonitoringPacket);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
