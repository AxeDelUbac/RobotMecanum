#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <MPU6050.h>

    void Imu_begin(void);
    void Imu_getAccelerometer(float fAcceleration[3]);
    void Imu_getGyroscope(float fGyroscope[3]);

#endif
