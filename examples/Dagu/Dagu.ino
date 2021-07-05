#include <Arduino.h>
#include <RotaryEncoder.h>
#include <PID_v1.h>
#include <L298N.h>

char A_IN1 = 8, A_IN2 = 9, A_EN = 17, A_INT1 = 4, A_INT2 = 5;
char B_IN1 = 10, B_IN2 = 11, B_EN = 16, B_INT1 = 2, B_INT2 = 3;

double SetpointLeft, OutputLeft;
double InputLeft = 0;
double SetpointRight, OutputRight;
double InputRight = 0;
double Kp=1.3, Ki=15, Kd=0.01;

double last_time_left = millis();
double tick_time_left = 0;
long old_position_left = 0;

double last_time_right = millis();
double tick_time_right = 0;
long old_position_right = 0

//dagu wheel radius = 31mm circumfrance 0.19478, 125 encoder ticks per rev

RotaryEncoder encoder_left(A_INT1, A_INT2, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder encoder_right(B_INT1, B_INT2, RotaryEncoder::LatchMode::TWO03);
L298N left(A_IN1, A_IN2, A_EN);
L298N right(B_IN1, B_IN2, B_EN);
PID PID_left(&InputLeft, &OutputLeft, &SetpointLeft, Kp, Ki, Kd, DIRECT);
PID PID_right(&InputRight, &OutputRight, &SetpointRight, Kp, Ki, Kd, DIRECT);


void CheckPositionLeft(){
  encoder_left.tick();
}

void CheckPositionRight(){
  encoder_right.tick();
}

double calculate_rpm_left(){
  long new_position_left = encoder_left.getPosition();
  long position_change_left;
  double RPM_left;

  if (new_position_left != old_position_left) {
    tick_time_left = (millis() - last_time_left);
    position_change_left = old_position_left - new_position_left;
    RPM_left = 1 / ((double(tick_time_left / position_change_left) * 125)/1000/60); //10041 18538 = ticks per rev, 1 rev = 42.73cm
    old_position_left = new_position_left;
    last_time_left = millis();   
  }
  else{
    RPM_left = 0.0;
  }
  delay(20);
  return RPM_left;
}

double calculate_rpm_right(){
  long new_position_right = encoder_right.getPosition();
  long position_change_right;
  double RPM_right;

  if (new_position_right != old_position_right) {
    tick_time_right = (millis() - last_time_right);
    position_change_right = old_position_right - new_position_right;
    RPM_left = 1 / ((double(tick_time_right / position_change_right) * 125)/1000/60); //10041 18538 = ticks per rev, 1 rev = 42.73cm
    old_position_right = new_position_right;
    last_time_right = millis();   
  }
  else{
    RPM_right = 0.0;
  }
  delay(20);
  return RPM_right;
}



//double covert_vel_rpm(double vel){
//  double RPM; 
//  RPM = (vel / wheel_circumfrance) * 60;
//  return RPM;
//}



void setup(){
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(A_INT1), CheckPositionLeft, CHANGE);
  pinMode(A_INT1, INPUT);
  attachInterrupt(digitalPinToInterrupt(A_INT2), CheckPositionLeft, CHANGE);
  pinMode(A_INT2, INPUT);
  attachInterrupt(digitalPinToInterrupt(B_INT1), CheckPositionRight, CHANGE);
  pinMode(B_INT1, INPUT);
  attachInterrupt(digitalPinToInterrupt(B_INT2), CheckPositionRight, CHANGE);
  pinMode(B_INT2, INPUT);

  SetpointLeft = 0;
  SetpointRight = 0;
  PID_left.SetSampleTime(20);
  PID_left.SetMode(AUTOMATIC);
  PID_left.SetOutputLimits(0,255);
  PID_right.SetSampleTime(20);
  PID_right.SetMode(AUTOMATIC);
  PID_right.SetOutputLimits(0,255);
} // setup()


// Read the current position of the encoder and print out when changed.
void loop(){
  SetpointLeft = 20.0;//covert_vel_rpm(vel);
  SetpointRight = 20.0;
  InputLeft = calculate_rpm_left();
  InputRight = calculate_rpm_right();
  PID_left.Compute();
  PID_right.Compute();


  if (SetpointLeft > 0.0){
    left.forwards(char(OutputLeft)); 
  }
  else{
    left.backwards(char(OutputLeft));
  }

  if (SetpointRight > 0.0){
    right.forwards(char(OutputRight)); 
  }
  else{
    right.backwards(char(OutputRight));
  }
//    Serial.print("Output_PWM:");
//    Serial.print(Output);
//    Serial.print(",");
//    Serial.print("Input_RPM:");
//    Serial.print(Input);
//    Serial.print(",");
//    Serial.print("Setpoint:");
//    Serial.println(Setpoint);
       
} // loop ()