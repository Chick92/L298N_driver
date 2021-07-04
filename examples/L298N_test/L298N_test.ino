#include <L298N.h>
#include <RotaryEncoder.h>
#include <PID_v1.h>

char A_IN1 = 8;
char A_IN2 = 9;
char A_EN = 17;
char A_INT1 = 3;
char A_INT2 = 4;
char B_IN1 = 10;
char B_IN2 = 11;
char B_EN = 16;

L298N left(A_IN1, A_IN2, A_EN, A_INT1, A_INT2);

void setup()
{
}

void loop(){
  left.set_rpm(20);
}

