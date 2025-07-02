#ifndef GPIO_H
#define GPIO_H

#include <Arduino.h>

//right back motor
#define leftFrontMotorHigh PE15
#define leftFrontMotorLow PB10
#define leftFrontMotorPWM PB11

//left back motor
#define rightFrontMotorHigh PE14
#define rightFrontMotorLow PE12
#define rightFrontMotorPWM PE10

//right front motor
#define leftBackMotorHigh PB13
#define leftBackMotorLow PB15
#define leftBackMotorPWM PC6

//left front motor
#define rightBackMotorHigh PA15
#define rightBackMotorLow PC7
#define rightBackMotorPWM PB5


#define rightFrontFirstHallSensor PE4
#define rightFrontSecondHallSensor PE5

#define leftBackFirstHallSensor PE6
#define leftBackSecondHallSensor PE3

#define rightBackFirstHallSensor PF8
#define rightBackSecondHallSensor PF7

#define leftFrontFirstHallSensor PF9
#define leftFrontSecondHallSensor PG1

void GPIO_init(void);

#endif // GPIO_H