#include "motorGearBox.h"

motorGearBox::motorGearBox(int highPin, int lowPin, int pwmPin){
    this->highPin = highPin;
    this->lowPin = lowPin;
    this->pwmPin = pwmPin;

}

  void motorGearBox::setMotorDirectionPWM(bool iDirection, int iSpeedInPWM){
    
    if (iDirection == 1) // avant
    {
      digitalWrite(this->highPin, HIGH); 
      digitalWrite(this->lowPin, LOW);
    }
    else if (iDirection == 0)
    {
      digitalWrite(this->highPin, LOW); 
      digitalWrite(this->lowPin, HIGH);
    }
    analogWrite(this->pwmPin, abs(iSpeedInPWM));
  }

  void motorGearBox::stopMotor(void){
    digitalWrite(this->highPin, HIGH); 
    digitalWrite(this->lowPin, HIGH);
  }

void motorGearBox::setMotorDirectionPercent(bool iDirection, int iSpeedInPercent) {
    int PWMSpeed = iSpeedInPercent * 255 / 100;
    setMotorDirectionPWM(iDirection, PWMSpeed);
}