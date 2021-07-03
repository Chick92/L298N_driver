/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/
#ifndef L298N_h
#define L298N_h

#include "Arduino.h"
#include <RotaryEncoder.h>
#include <PID_v1.h>

class L298N
{
  public:
    L298N(char IN1, char IN2, char EN, char INT1, char INT2);
	void set_rpm(int rpm);
	
  private:
    char _IN1, _IN2, _EN, _INT1, _INT2;
	double Input, Output, Setpoint, Kp, Ki, Kd;
	double last_time, tick_time;
	long old_position;
	
	RotaryEncoder encoder(_INT1, _INT2, RotaryEncoder::LatchMode::TWO03);
	PID PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
	
	void checkPosition();
	double calculate_rpm();
	void forwards(char pwm);
	void backwards(char pwm);
};

#endif
