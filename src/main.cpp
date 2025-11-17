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
  xTaskCreate(commandProcessingTask, "commandProcessingTask", 256, NULL, 1, NULL);
  xTaskCreate(IMUTask,"IMUTask", 256, NULL, 1, NULL);
  xTaskCreate(UartTask,"UartTask", 256, NULL, 1, NULL);
}

void System_componentsInit(void){
  GPIO_init();
  GlobalControl_init(&tGlobalControl);
  MovementController_init(&tMovementController);
  CommandProcessing_init();
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
    // BluetoothReception_debug();

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

        fsetpointRpm = 0;//CommandProcessing_modifySetpointInRpm();

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

/* 
 * UartTask - Tâche pour transmission des données via UART4
 */
void UartTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(15000); // 1 Hz (toutes les 1000ms)
  
  while (1) {
    // Préparation du paquet de données
    MotorDataPacket_t motorData = {};
    for (int i = 0; i < 4; i++) {
      motorData.wheelSpeedRPM[i] = fWheelSpeed[i];
      motorData.motorPower[i] = PIDoutput[i];
    }

    // DataPacket_t completeData = {};
    
    // // Remplissage avec des données de test
    // for (int i = 0; i < 4; i++) {
    //   completeData.motorData.wheelSpeedRPM[i] = fWheelSpeed[i];
    //   completeData.motorData.motorPower[i] = PIDoutput[i];
    // }
    
    // completeData.odometryData.positionX = robotOdometry.pose.x;
    // completeData.odometryData.positionY = robotOdometry.pose.y;
    // completeData.odometryData.orientation = robotOdometry.pose.theta;

    // for (int i = 0; i < 3; i++) {
    //   completeData.imuData.accel[i]=0;
    //   completeData.imuData.gyro[i]=0;
    //   completeData.imuData.mag[i]=0;
    // }
    // completeData.imuData.roll=0;
    // completeData.imuData.pitch=0;
    // completeData.imuData.yaw =0;
    

  // Transmission via notre transmetteur
  SerialDataTransmitter_sendCompleteData(&uartTransmitter, &motorData);

  SerialDataReceiver_process(&UartSerial, &lastMonitoringPacket);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
