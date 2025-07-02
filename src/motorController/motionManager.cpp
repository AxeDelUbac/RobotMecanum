#include "motionManager.h"

int state = 0;

void MotionManager::GoUpBack() {
    if (Serial.available() > 0)
    {
        state = Serial.parseInt();
        if (state > 0) // avant
        {
            oMovementController.movementFront(state);
            Serial.print("Avant ");
        }
        else if (state < 0) // arrière
        {
            oMovementController.movementBack(state);
            Serial.print("Arriere ");
        }
        else // Stop (freinage)
        {
            oMovementController.movementstop();
        }
    }
}