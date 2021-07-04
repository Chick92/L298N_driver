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
    L298N(char IN1, char IN2, char EN, char A_INT1, char A_INT2);
	void set_rpm(int rpm);
	
  private:
    char _IN1, _IN2, _EN, _INT1, _INT2;
	
	double last_time = millis();
	double tick_time = 0;
	long old_position = 0;

	double Setpoint, Output;
	double Input = 0;
	double Kp=1.3, Ki=15, Kd=0.01;

  RotaryEncoder encoder(char A_INT1, char A_INT2, RotaryEncoder::LatchMode::TWO03);
  PID pid(double &Input, double &Output, double &Setpoint, double Kp, double Ki, double Kd, char DIRECT);	
	
	void checkPosition();
	double calculate_rpm();
	void forwards(char pwm);
	void backwards(char pwm);
};

#endif

/*
Arduino: 1.8.15 (Linux), Board: "Arduino Uno"


In file included from /home/ubuntu/arduino-1.8.15/hardware/tools/avr/avr/include/avr/io.h:272:0,
                 from /home/ubuntu/arduino-1.8.15/hardware/tools/avr/avr/include/avr/pgmspace.h:90,
                 from /home/ubuntu/arduino-1.8.15/hardware/arduino/avr/cores/arduino/Arduino.h:28,
                 from sketch/L298N_test.ino.cpp:1:
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:16:45: error: expected ',' or '...' before numeric constant
     L298N(char IN1, char IN2, char EN, char INT1, char INT2);
                                             ^
In file included from /home/ubuntu/Arduino/libraries/L298N_driver/examples/L298N_test/L298N_test.ino:1:0:
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:25:24: error: '_INT1' is not a type
  RotaryEncoder encoder(_INT1, _INT2, RotaryEncoder::LatchMode::TWO03);
                        ^~~~~
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:25:31: error: '_INT2' is not a type
  RotaryEncoder encoder(_INT1, _INT2, RotaryEncoder::LatchMode::TWO03);
                               ^~~~~
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:25:64: error: 'RotaryEncoder::LatchMode::TWO03' is not a type
  RotaryEncoder encoder(_INT1, _INT2, RotaryEncoder::LatchMode::TWO03);
                                                                ^~~~~
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:26:10: error: expected identifier before '&' token
  PID PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
          ^
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:26:18: error: expected identifier before '&' token
  PID PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
                  ^
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:26:27: error: expected identifier before '&' token
  PID PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
                           ^
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:26:38: error: 'Kp' is not a type
  PID PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
                                      ^~
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:26:42: error: 'Ki' is not a type
  PID PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
                                          ^~
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:26:46: error: 'Kd' is not a type
  PID PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
                                              ^~
In file included from /home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:11:0,
                 from /home/ubuntu/Arduino/libraries/L298N_driver/examples/L298N_test/L298N_test.ino:1:
/home/ubuntu/Arduino/libraries/PID/PID_v1.h:14:19: error: expected identifier before numeric constant
   #define DIRECT  0
                   ^
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:26:50: note: in expansion of macro 'DIRECT'
  PID PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
                                                  ^~~~~~
/home/ubuntu/Arduino/libraries/PID/PID_v1.h:14:19: error: expected ',' or '...' before numeric constant
   #define DIRECT  0
                   ^
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:26:50: note: in expansion of macro 'DIRECT'
  PID PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
                                                  ^~~~~~
In file included from /home/ubuntu/Arduino/libraries/L298N_driver/examples/L298N_test/L298N_test.ino:1:0:
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:26:56: warning: declaration of 'PID L298N::PID(int&, int&, int&, int, int, int, int)' [-fpermissive]
  PID PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
                                                        ^
In file included from /home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:11:0,
                 from /home/ubuntu/Arduino/libraries/L298N_driver/examples/L298N_test/L298N_test.ino:1:
/home/ubuntu/Arduino/libraries/PID/PID_v1.h:5:7: warning: changes meaning of 'PID' from 'class PID' [-fpermissive]
 class PID
       ^~~
L298N_test:12:46: error: no matching function for call to 'L298N::L298N(char&, char&, char&, char&, char&)'
 L298N left(A_IN1, A_IN2, A_EN, A_INT1, A_INT2);
                                              ^
In file included from /home/ubuntu/Arduino/libraries/L298N_driver/examples/L298N_test/L298N_test.ino:1:0:
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:16:5: note: candidate: L298N::L298N(char, char, char, char)
     L298N(char IN1, char IN2, char EN, char INT1, char INT2);
     ^~~~~
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:16:5: note:   candidate expects 4 arguments, 5 provided
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:13:7: note: candidate: constexpr L298N::L298N(const L298N&)
 class L298N
       ^~~~~
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:13:7: note:   candidate expects 1 argument, 5 provided
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:13:7: note: candidate: constexpr L298N::L298N(L298N&&)
/home/ubuntu/Arduino/libraries/L298N_driver/L298N.h:13:7: note:   candidate expects 1 argument, 5 provided
exit status 1
no matching function for call to 'L298N::L298N(char&, char&, char&, char&, char&)'


This report would have more information with
"Show verbose output during compilation"
option enabled in File -> Preferences.
*/
