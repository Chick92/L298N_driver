#include <L298N.h>

char A_IN1 = 8;
char A_IN2 = 9;
char A_EN = 17;
char B_IN1 = 10;
char B_IN2 = 11;
char B_EN = 16;

L298N left(A_IN1, A_IN2, A_EN);
L298N right(B_IN1, B_IN2, B_EN);

void setup()
{
}

void loop()
{
  left.forwards(255);
  right.forwards(255);
  delay(3000);
  left.backwards(255);
  right.backwards(255);
  delay(3000);
}

