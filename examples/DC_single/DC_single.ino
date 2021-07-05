#include <Arduino.h>
#include <RotaryEncoder.h>
#include <PID_v1.h>
#include <L298N.h>

char A_IN1 = 8;
char A_IN2 = 9;
char A_EN = 17;
char B_IN1 = 10;
char B_IN2 = 11;
char B_EN = 16;
char PIN_IN1 = 4; //4 2
char PIN_IN2 = 5; //5 3

double Setpoint, Output;
double Input = 0;
double Kp=1.3, Ki=15, Kd=0.01;

double last_time = millis();
double tick_time = 0;
long old_position = 0;

//dagu wheel radius = 31mm circumfrance 0.19478, 333 encoder ticks per rev

RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);
L298N left(A_IN1, A_IN2, A_EN);
L298N right(B_IN1, B_IN2, B_EN);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void checkPosition()
{
  encoder.tick(); // just call tick() to check the state.
}


double calculate_rpm(){
   long new_position = encoder.getPosition();
   long position_change;
   double RPM;

   if (new_position != old_position) {
       tick_time = (millis() - last_time);
       position_change = old_position - new_position;
       RPM = 1 / ((double(tick_time / position_change) * 125)/1000/60); //10041 18538 = ticks per rev, 1 rev = 42.73cm
       old_position = new_position;
       last_time = millis();   
   }
   else{
       RPM = 0.0;
   }
   delay(20);
   return RPM;
}



//double covert_vel_rpm(double vel){
//  double RPM; 
//  RPM = (vel / wheel_circumfrance) * 60;
//  return RPM;
//}



void setup()
{
  Serial.begin(115200);
  while (! Serial);
  //Serial.println("InterruptRotator example for the RotaryEncoder library.");

  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  pinMode(PIN_IN1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);
  pinMode(PIN_IN2, INPUT);
   Setpoint = 0;
   myPID.SetSampleTime(20);
   myPID.SetMode(AUTOMATIC);
   myPID.SetOutputLimits(0,255);
} // setup()


// Read the current position of the encoder and print out when changed.
void loop(){
  Setpoint = 20.0;//covert_vel_rpm(vel);
  Input = calculate_rpm();
  myPID.Compute();


  if (Setpoint > 0.0){
    left.forwards(char(Output)); 
  }
  else{
    left.backwards(char(Output));
  }
      Serial.print("Output_PWM:");
    Serial.print(Output);
    Serial.print(",");
    Serial.print("Input_RPM:");
    Serial.print(Input);
    Serial.print(",");
    Serial.print("Setpoint:");
    Serial.println(Setpoint);
       
} // loop ()