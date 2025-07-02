#include "GPIO.h"

void GPIO_init(void){

    // Front right motor
    pinMode(rightFrontMotorHigh, OUTPUT);
    pinMode(rightFrontMotorLow, OUTPUT);
    pinMode(rightFrontMotorPWM, OUTPUT);
    pinMode(rightFrontFirstHallSensor, INPUT);
    pinMode(rightFrontSecondHallSensor, INPUT);

    // Front left motor
    pinMode(leftFrontMotorHigh, OUTPUT);
    pinMode(leftFrontMotorLow, OUTPUT);
    pinMode(leftFrontMotorPWM, OUTPUT);
    pinMode(leftFrontFirstHallSensor, INPUT);
    pinMode(leftFrontSecondHallSensor, INPUT);

    // back right motor
    pinMode(rightBackMotorHigh, OUTPUT);
    pinMode(rightBackMotorLow, OUTPUT);
    pinMode(rightBackMotorPWM, OUTPUT);

    // back left motor
    pinMode(leftBackMotorHigh, OUTPUT);
    pinMode(leftBackMotorLow, OUTPUT);
    pinMode(leftBackMotorPWM, OUTPUT);

}