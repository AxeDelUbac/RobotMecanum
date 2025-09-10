#include "motorGearBox.h"

void MotorGearBox_init(motorGearBox_t* mg, int highPin, int lowPin, int pwmPin){
    if (!mg) return;
    mg->highPin = highPin;
    mg->lowPin = lowPin;
    mg->pwmPin = pwmPin;
}

void MotorGearBox_setMotorDirectionPWM(motorGearBox_t* mg, bool iDirection, int iSpeedInPWM){
  if (!mg) return;
  if (iDirection == 1) // avant
  {
    digitalWrite(mg->highPin, HIGH); 
    digitalWrite(mg->lowPin, LOW);
  }
  else if (iDirection == 0)
  {
    digitalWrite(mg->highPin, LOW); 
    digitalWrite(mg->lowPin, HIGH);
  }
  analogWrite(mg->pwmPin, abs(iSpeedInPWM));
}

void MotorGearBox_stopMotor(motorGearBox_t* mg){
  if (!mg) return;
  digitalWrite(mg->highPin, HIGH); 
  digitalWrite(mg->lowPin, HIGH);
}

void MotorGearBox_setMotorDirectionPercent(motorGearBox_t* mg, bool iDirection, int iSpeedInPercent) {
    if (!mg) return;
    int PWMSpeed = iSpeedInPercent * 255 / 100;
    MotorGearBox_setMotorDirectionPWM(mg, iDirection, PWMSpeed);
}