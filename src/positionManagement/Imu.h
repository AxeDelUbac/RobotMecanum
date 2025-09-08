#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <MPU6050.h>

class Imu {
    public:

        void begin(void);
        void getAccelerometer(float fAcceleration[3]);
        void getGyroscope(float fGyroscope[3]);

    private:
        float fAcceleration[3];
        float fGyroscope[3];
        MPU6050 mpu6050;
};

#endif
