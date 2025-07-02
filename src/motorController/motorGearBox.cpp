#include "motorGearBox.h"

motorGearBox::motorGearBox(int highPin, int lowPin, int pwmPin){
    this->highPin = highPin;
    this->lowPin = lowPin;
    this->pwmPin = pwmPin;

}

  void motorGearBox::setMotorDirection(bool direction, int pourcentSpeed){
    int PWMSpeed = pourcentSpeed * 255 / 100; // Convertir la vitesse en pourcentage en valeur PWM 
    
    if (direction == 1) // avant
    {
      digitalWrite(this->highPin, HIGH); 
      digitalWrite(this->lowPin, LOW);
    }
    else if (direction == 0)
    {
      digitalWrite(this->highPin, LOW); 
      digitalWrite(this->lowPin, HIGH);
    }
    analogWrite(this->pwmPin, abs(PWMSpeed));
  }

  void motorGearBox::stopMotor(){
    digitalWrite(this->highPin, HIGH); 
    digitalWrite(this->lowPin, HIGH);
  }