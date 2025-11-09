#include "main.h"

GlobalControl tGlobalControl;
movementController_t tMovementController;
MecanumOdometry_t robotOdometry;

// Variables pour la communication de données UART
SerialDataTransmitter_t uartTransmitter;
HardwareSerial UartComm(PC11, PC10);  // UART3 pour transmission de données (nom différent)


void task_create(void){
  Serial.println("Init FreeRTOS Task...");

  xTaskCreate(displayInformationTask, "displayInformationTask", 256, NULL, 1, NULL);
  xTaskCreate(MotorRegulationTask, "MotorRegulationTask", 512, NULL, 2, NULL);
  xTaskCreate(speedMesurementTask, "rotaryEncoderTask", 256, NULL, 1, NULL);
  xTaskCreate(commandProcessingTask, "commandProcessingTask", 256, NULL, 1, NULL);
  xTaskCreate(IMUTask,"IMUTask", 256, NULL, 1, NULL);
  xTaskCreate(DataTransmissionTask,"DataTransmissionTask", 512, &uartTransmitter, 1, NULL);
}

void System_componentsInit(void){
  GPIO_init();
  GlobalControl_init(&tGlobalControl);
  MovementController_init(&tMovementController);
  CommandProcessing_init();
  BluetoothReception_init();
  PositionOrientation_init();
  Imu_init();
  
  // Initialisation de l'odométrie
  float wheelRadius = wheelDiameter / 2.0f; // Utilise la constante du fichier encoderParameter.h
  MecanumOdometry_init(&robotOdometry, wheelRadius, 0.20f, 0.18f); // Ajustez les dimensions selon votre robot
  
  // Initialisation du transmetteur UART avec configuration complète
  SerialDataTransmitter_init(&uartTransmitter, &UartComm, 115200, 20, true); // 115200 bauds, 20Hz, checksum activé
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

    GlobalSpeed_debug();
    // Imu_SerialDebug();
    MecanumOdometry_debug(&robotOdometry);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void MotorRegulationTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz pour asservissement + odométrie
  
  while (1) {
    // === ASSERVISSEMENT MOTEUR ===
    GlobalControl_UpdateSetpoint(&tGlobalControl, fsetpointRpm, fWheelSpeed, PIDoutput);
    MovementController_setMotorSpeedInPWM(&tMovementController, PIDoutput);
    MovementController_movementFront(&tMovementController);

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

void commandProcessingTask(void *pvParameters) {
  while (1) {

        fsetpointRpm = CommandProcessing_modifySetpointInRpm();

    vTaskDelay(pdMS_TO_TICKS(50));
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

void DataTransmissionTask(void *pvParameters) {
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100); // Conversion Hz -> ms
  
  Serial.println("DataTransmissionTask started");
  
  while (1) {
    // Préparation du paquet complet avec toutes les données
    RobotCompleteDataPacket_t robotCompleteDataPacket_t = {}; // Initialisation à zéro
    
    // === DONNÉES MOTEURS ===
    for (int i = 0; i < 4; i++) {
      robotCompleteDataPacket_t.motorData.wheelSpeedRPM[i] = fWheelSpeed[i];
      robotCompleteDataPacket_t.motorData.motorPower[i] = PIDoutput[i];
    }
    // === DONNÉES ODOMÉTRIE ===
    robotCompleteDataPacket_t.odometryData.positionX = robotOdometry.pose.x;
    robotCompleteDataPacket_t.odometryData.positionY = robotOdometry.pose.y;
    robotCompleteDataPacket_t.odometryData.orientation = robotOdometry.pose.theta;
    robotCompleteDataPacket_t.odometryData.velocityX = robotOdometry.velocity.vx;
    robotCompleteDataPacket_t.odometryData.velocityY = robotOdometry.velocity.vy;
    robotCompleteDataPacket_t.odometryData.angularVelocity = robotOdometry.velocity.omega;
    // === DONNÉES IMU ===
    for (int i = 0; i < 3; i++) {
      robotCompleteDataPacket_t.imuData.accel[i] = 0.0f;  // À remplacer par vos données IMU
      robotCompleteDataPacket_t.imuData.gyro[i] = 0.0f;   // À remplacer par vos données IMU  
      robotCompleteDataPacket_t.imuData.mag[i] = 0.0f;    // À remplacer par vos données IMU
    }
    robotCompleteDataPacket_t.imuData.roll = 0.0f;   // À remplacer par vos données IMU
    robotCompleteDataPacket_t.imuData.pitch = 0.0f;  // À remplacer par vos données IMU
    robotCompleteDataPacket_t.imuData.yaw = 0.0f;    // À remplacer par vos données IMU
    
    // === MÉTADONNÉES DU PAQUET ===
    robotCompleteDataPacket_t.timestamp = millis();
    robotCompleteDataPacket_t.packetId = robotCompleteDataPacket_t.packetId++;
    
    // === TRANSMISSION ===
    bool bsendSuccess =SerialDataTransmitter_sendCompleteData(&uartTransmitter, &robotCompleteDataPacket_t);
    if (bsendSuccess = true) {
      // Transmission réussie - debug optionnel
      static uint16_t debugCounter = 0;
      if (++debugCounter % 100 == 0) { // Affichage tous les 100 paquets (5 secondes à 20Hz)
        Serial.print("Transmitted packet #"); Serial.println(robotCompleteDataPacket_t.packetId);
      }
    } 
    else {
      Serial.println("Error: Failed to transmit data packet");
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

 