#ifndef CAPTEURHALL_H
#define CAPTEURHALL_H

class HallSensor {
  public:
    float getHallAngularSpeed();
    int getPulses();
    
    unsigned int hallPulses;

  private:
    
    float angularSpeed;
    
};

#endif