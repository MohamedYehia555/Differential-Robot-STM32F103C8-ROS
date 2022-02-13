#ifndef MotoCytron_h
#define MotoCytron_h
#include <Arduino.h>
#include "pin_assign.h"
#define MAXPWM 255  // The AVR can only do 8-bits
#define BRAKEVCC 0
#define CW       1
#define CCW      2
#define BRAKEGND 3
#define CS_THRESHOLD 100
class MotoCytron
{
  public:
    MotoCytron(){}
    void setup();
    void motorOff(byte motor);
    void motorGo(byte motor, byte direct, int pwm);
   
  private:
    const byte DIR1 = PB4;
    const byte PWM1 = PB8;
    const byte DIR2 = PB3;
    const byte PWM2 = PB9;
};
#endif
