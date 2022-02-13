
// Encoder_CFG.h
// Library for using rotary encoders with STM32 "Blue Pill" in Arduino environment.

#ifndef Encoder_CFG_H
#define Encoder_CFG_H
#include "Arduino.h"
class Encoder_CFG
{
  public:
    Encoder_CFG(int pin1, int pin2);
    void setup();
    // get the current position
    long  getPos() {
      return _position;
    }
    void setPos(long newPosition) {
      _position = newPosition;
    }

    void tick();

  private:
    int _pin1, _pin2;
    volatile byte _oldState;
    volatile long _position;
};

#endif /* Encoder_CFG_H */
