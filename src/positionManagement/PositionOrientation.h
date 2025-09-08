#ifndef POSITIONORIENTATION_H
#define POSITIONORIENTATION_H

#include "Imu.h"

class PositionOrientation {
    public:
        PositionOrientation();
        void begin();
        void update(float dt);
        void getEulerAngles(float& roll, float& pitch, float& yaw);

    private:

        Imu imu;
        float roll;
        float pitch;
        float yaw;
};

#endif
