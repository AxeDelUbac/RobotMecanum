#include "Imu.h"
#include <Wire.h>

  MPU6050 mpu6050;

void Imu_begin(void)
{
  Wire.begin();
  mpu6050.initialize();
}

void Imu_getAccelerometer(float fAcceleration[3])
{
   int16_t ax, ay, az;
    mpu6050.getAcceleration(&ax, &ay, &az);
    // Conversion en g (1g = 16384 pour MPU6050 par défaut)
    fAcceleration[0] = ax / 16384.0f;
    fAcceleration[1] = ay / 16384.0f;
    fAcceleration[2] = az / 16384.0f;
}

void Imu_getGyroscope(float fGyroscope[3])
{
    int16_t gx, gy, gz;
    mpu6050.getRotation(&gx, &gy, &gz);
    // Conversion en deg/s (1°/s = 131 pour MPU6050 par défaut)
    fGyroscope[0] = gx / 131.0f;
    fGyroscope[1] = gy / 131.0f;
    fGyroscope[2] = gz / 131.0f;
}