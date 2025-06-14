#include "motionManager.h"
#include "movementController.h"

int state = 0;

movementController movementController;

void MotionManager::GoUpBack() {
    if (Serial.available() > 0)
    {
        state = Serial.parseInt();
        if (state > 0) // avant
        {
            movementController.movementFront(state);
            Serial.print("Avant ");
        }
        else if (state < 0) // arrière
        {
            movementController.movementBack(state);
            Serial.print("Arriere ");
        }
        else // Stop (freinage)
        {
            movementController.movementstop();
        }
    }
}