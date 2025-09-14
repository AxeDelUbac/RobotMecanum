#include "Imu.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_MMC56x3.h>
#include "LSM6DS3Sensor.h"

LSM6DS3Sensor accgyr(&Wire, 0x6A << 1); // 0xD4
Adafruit_LIS3MDL mag;
Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);

ImuData_t imuData;
ImuOrientation_t imuOrientation;

static bool mmcInitOk = false;

// Assumptions about sensor full-scale / sensitivity:
// - Accelerometer assumed at +/-4g -> sensitivity ~0.122 mg/LSB = 0.000122 g/LSB
//   1 g = 9.80665 m/s^2 so accel_scale_m_s2 = 0.000122 * 9.80665
// - Gyroscope assumed at +/-2000 dps -> sensitivity ~70 mdps/LSB = 0.07 dps/LSB
// If your configuration uses different FS, replace these scales with actual values.
const float IMU_ACCEL_SCALE_G = 0.000122f; // g per LSB (example for +-4g)
const float IMU_G_TO_M_S2 = 9.80665f;
const float IMU_ACCEL_SCALE_M_S2 = IMU_ACCEL_SCALE_G * IMU_G_TO_M_S2;
const float IMU_GYRO_SCALE_DEG_S = 0.07f; // degrees/s per LSB (example for +-2000 dps)
const float IMU_DEG_TO_RAD = 3.14159265358979323846f / 180.0f;



void Imu_init(void)
{
  // utiliser l'instance globale `accgyr` définie plus haut
  LSM6DS3StatusTypeDef status = accgyr.begin();
  if (status == LSM6DS3_STATUS_OK) {
    Serial.println("accgyr.begin() OK");
  } else {
    Serial.print("accgyr.begin() ERROR: "); Serial.println((int)status);
  }

  // lire l'ID du capteur
  uint8_t who = 0;
  if (accgyr.ReadID(&who) == LSM6DS3_STATUS_OK) {
    Serial.print("LSM6 ReadID = 0x"); if (who < 0x10) Serial.print("0"); Serial.println(who, HEX);
  } else {
    Serial.println("LSM6 ReadID failed");
  }

  // activer capteurs (accel + gyro)
  if (accgyr.Enable_X() == LSM6DS3_STATUS_OK) Serial.println("Accel enabled");
  else Serial.println("Enable_X failed");
  if (accgyr.Enable_G() == LSM6DS3_STATUS_OK) Serial.println("Gyro enabled");
  else Serial.println("Enable_G failed");
  delay(50);

}

void Imu_getAccelerometer(void)
{
  LSM6DS3StatusTypeDef s = accgyr.Get_X_Axes(imuData.i32Acceleration);
}
void Imu_getGyroscope(void)
{
  LSM6DS3StatusTypeDef s = accgyr.Get_G_Axes(imuData.i32Gyroscope);
}

int32_t Imumag[3]; // to hold magnetometer readings
void Imu_getMagnetometer(void)
{

}

void Imu_getAcceleration_m_s2(void)
{
  // Ensure raw data is up to date by reading sensor
  Imu_getAccelerometer();
  for (int i = 0; i < 3; i++) {
    float v = imuData.i32Acceleration[i] * IMU_ACCEL_SCALE_M_S2;
    imuData.fAcceleration_m_s2[i] = v;
  }
}

void Imu_getAngularRate_deg_s(float out_deg_s[3])
{
  // Ensure raw data is up to date by reading sensor
  Imu_getGyroscope();
  for (int i = 0; i < 3; i++) {
    out_deg_s[i] = imuData.i32Gyroscope[i] * IMU_GYRO_SCALE_DEG_S;
  }
}

void Imu_getAngularRate_rad_s(void)
{
  float deg[3];
  Imu_getAngularRate_deg_s(deg);
  for (int i = 0; i < 3; i++) {
    float r = deg[i] * IMU_DEG_TO_RAD;
    imuData.fGyroscope_rad_s[i] = r;
  }
}

void Imu_updateOrientation(float dt)
{
  // Lire capteurs (assure que les valeurs float sont à jour)
  Imu_getAcceleration_m_s2();   // remplit imuData.fAcceleration_m_s2
  Imu_getAngularRate_rad_s();   // remplit imuData.fGyroscope_rad_s
  // Optionnel: Imu_getMagnetometer(); // remplir imuData.i32Magnetometer si dispo

  // Accélération en m/s2
  float ax = imuData.fAcceleration_m_s2[0];
  float ay = imuData.fAcceleration_m_s2[1];
  float az = imuData.fAcceleration_m_s2[2];

  // Vitesses angulaires (rad/s)
  float gx = imuData.fGyroscope_rad_s[0];
  float gy = imuData.fGyroscope_rad_s[1];
  float gz = imuData.fGyroscope_rad_s[2];


}


void Imu_SerialDebug(void)
{

  Serial.print(">Accelerometer XAxis:");
  Serial.println(imuData.i32Acceleration[0]);
  Serial.print(">Accelerometer YAxis:");
  Serial.println(imuData.i32Acceleration[1]);
  Serial.print(">Accelerometer ZAxis:");
  Serial.println(imuData.i32Acceleration[2]);

  Serial.print("> Gyroscope XAxis:");
  Serial.println(imuData.i32Gyroscope[0]);
  Serial.print("> Gyroscope YAxis:");
  Serial.println(imuData.i32Gyroscope[1]);
  Serial.print("> Gyroscope ZAxis:");
  Serial.println(imuData.i32Gyroscope[2]);

  Serial.print(">Magnetometer XAxis:");
  Serial.println(imuData.i32Magnetometer[0]);
  Serial.print(">Magnetometer YAxis:");
  Serial.println(imuData.i32Magnetometer[1]);
  Serial.print(">Magnetometer ZAxis:");
  Serial.println(imuData.i32Magnetometer[2]);

  Serial.print(">Acceleration XAxis:");
  Serial.println(imuData.fAcceleration_m_s2[0]);
  Serial.print(">Acceleration YAxis:");
  Serial.println(imuData.fAcceleration_m_s2[1]);
  Serial.print(">Acceleration ZAxis:");
  Serial.println(imuData.fAcceleration_m_s2[2]);

  Serial.print(">Angular speed XAxis:");
  Serial.println(imuData.fGyroscope_rad_s[0]);
  Serial.print(">Angular speed YAxis:");
  Serial.println(imuData.fGyroscope_rad_s[1]);
  Serial.print(">Angular speed ZAxis:");
  Serial.println(imuData.fGyroscope_rad_s[2]);
}

