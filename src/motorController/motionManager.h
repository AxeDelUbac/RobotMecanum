#ifndef MOTIONMANAGER_H
#define MOTIONMANAGER_H

#include <Arduino.h>
#include <GPIO.h>
// #include "main.h"

#include "movementController.h"

class MotionManager {
public:
    void GoUpBack();
    movementController oMovementController;
};

#endif
