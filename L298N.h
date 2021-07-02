/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/
#ifndef L298N_h
#define L298N_h

#include "Arduino.h"

class L298N
{
  public:
    L298N(char IN1, char IN2, char EN);
    void forwards(char pwm);
    void backwards(char pwm);
  private:
    char _IN1, _IN2, _EN;
};

#endif
