#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <stdint.h>

void Imu_init(void);
void Imu_getAccelerometer(void);
void Imu_getGyroscope(void);
void Imu_getMagnetometer(void);
void Imu_SerialDebug(void);
void Imu_updateOrientation(float dt);

// Conversions and helpers
// Returns acceleration in m/s^2 for axis [0]=X,[1]=Y,[2]=Z
void Imu_getAcceleration_m_s2(void);
// Returns angular rate in degrees/s for axis [0]=X,[1]=Y,[2]=Z
void Imu_getAngularRate_deg_s(float out_deg_s[3]);
// Returns angular rate in radians/s
void Imu_getAngularRate_rad_s(void);

typedef struct {
    int32_t i32Acceleration[3]; // raw axes (LSB)
    int32_t i32Gyroscope[3];    // raw axes (LSB)
    int32_t i32Magnetometer[3]; // raw axes (LSB)
    float fAcceleration_m_s2[3]; // converted to m/s^2
    float fGyroscope_rad_s[3];   // converted to rad/s
    float fMagnetometer_uT[3];   // converted to microTesla
} ImuData_t;

typedef struct{
    float froll;
    float fpitch;
    float fyaw;
} ImuOrientation_t;

#endif
