#include "IT.h"

volatile unsigned int iNombreTour[4] = {0, 0, 0, 0};
volatile unsigned int iDirection[4] = {0, 0, 0, 0};

void rightFrontMotorIT() 
{
  iNombreTour[0]++;
  if (digitalRead(rightFrontSecondHallSensor)== HIGH) {
    iDirection[0] = 1; // Clockwise
  }
  else 
  {
    iDirection[0] = -1; // Counterclockwise
  }
}

void leftFrontMotorIT() 
{
  iNombreTour[1]++;
  if (digitalRead(leftFrontSecondHallSensor)== HIGH) {
    iDirection[1] = 1; // Clockwise
  }
  else 
  {
    iDirection[1] = -1; // Counterclockwise
  }
}

void rightBackMotorIT() 
{
  iNombreTour[2]++;
  if (digitalRead(rightBackSecondHallSensor)== HIGH) {
    iDirection[2] = 1; // Clockwise
  }
  else 
  {
    iDirection[2] = -1; // Counterclockwise
  }
}

void leftBackMotorIT() 
{
  iNombreTour[3]++;
  if (digitalRead(rightBackSecondHallSensor)== HIGH) {
    iDirection[3] = 1; // Clockwise
  }
  else 
  {
    iDirection[3] = -1; // Counterclockwise
  }
}

void createIT(void) 
{
    attachInterrupt(digitalPinToInterrupt(rightFrontFirstHallSensor), rightFrontMotorIT, RISING);
    attachInterrupt(digitalPinToInterrupt(leftFrontFirstHallSensor), leftFrontMotorIT, RISING);
    attachInterrupt(digitalPinToInterrupt(rightBackFirstHallSensor), rightBackMotorIT, RISING);
    attachInterrupt(digitalPinToInterrupt(leftBackFirstHallSensor), leftBackMotorIT, RISING);
}