#ifndef POSITIONORIENTATION_H
#define POSITIONORIENTATION_H

#include "Imu.h"

    void PositionOrientation_begin();
    void PositionOrientation_update(float dt);
    void PositionOrientation_getEulerAngles(float roll, float pitch, float yaw);

    typedef struct {
        float roll;
        float pitch;
        float yaw;
    } EulerAngles;

#endif
