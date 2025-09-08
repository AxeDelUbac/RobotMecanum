#include "PositionOrientation.h"
#include <math.h>

PositionOrientation::PositionOrientation()
    : roll(0.0f), pitch(0.0f), yaw(0.0f)
{}

void PositionOrientation::begin() {
    imu.begin();
}

void PositionOrientation::update(float dt) {
    float accel[3], gyro[3];
    imu.getAccelerometer(accel);
    imu.getGyroscope(gyro);

    // Calcul des angles à partir de l'accéléromètre (en degrés)
    float roll_acc  = atan2(accel[1], accel[2]) * 180.0f / M_PI;
    float pitch_acc = atan2(-accel[0], sqrt(accel[1]*accel[1] + accel[2]*accel[2])) * 180.0f / M_PI;

    // Filtre complémentaire pour roll et pitch
    const float alpha = 0.98f;
    roll  = alpha * (roll + gyro[0] * dt) + (1.0f - alpha) * roll_acc;
    pitch = alpha * (pitch + gyro[1] * dt) + (1.0f - alpha) * pitch_acc;
    yaw  += gyro[2] * dt; // Yaw uniquement gyroscope (dérive possible)

    if (yaw > 180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;
}

void PositionOrientation::getEulerAngles(float& outRoll, float& outPitch, float& outYaw) {
    outRoll = roll;
    outPitch = pitch;
    outYaw = yaw;
}