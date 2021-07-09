#include <Arduino.h>
#include <RotaryEncoder.h>
#include <PID_v1.h>
#include <L298N.h>
#include <Adafruit_NeoPixel.h>


char A_IN1 = 8, A_IN2 = 9, A_EN = 17, A_INT1 = 4, A_INT2 = 5;
char B_IN1 = 10, B_IN2 = 11, B_EN = 16, B_INT1 = 3, B_INT2 = 2;
int AnPin1 = A1;


double SetpointLeft, OutputLeft;
double InputLeft = 0;
double SetpointRight, OutputRight;
double InputRight = 0;
double Kp=4, Ki=10, Kd=1;

double last_time_left = micros();
double tick_time_left = 0;
long old_position_left = 0;

double last_time_right = micros();
double tick_time_right = 0;
long old_position_right = 0;

double old_tick_duration_left = millis();
double tick_duration_left = 0.0;
double old_tick_duration_right = millis();
double tick_duration_right = 0.0;
double tick_output_left = 0, tick_output_right = 0;
char tick_counter_left = 0, tick_counter_right = 0;
double rpm_left = 0, rpm_right = 0;

int watchdog_right = 0;
int watchdog_left = 0;

//dagu wheel radius = 31mm circumfrance 0.19478, 125 encoder ticks per rev

RotaryEncoder encoder_left(A_INT1, A_INT2, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder encoder_right(B_INT1, B_INT2, RotaryEncoder::LatchMode::TWO03);
L298N left(A_IN1, A_IN2, A_EN);
L298N right(B_IN1, B_IN2, B_EN);
PID PID_left(&InputLeft, &OutputLeft, &SetpointLeft, Kp, Ki, Kd, DIRECT);
PID PID_right(&InputRight, &OutputRight, &SetpointRight, Kp, Ki, Kd, DIRECT);
Adafruit_NeoPixel pixels(1, 28, NEO_GRB + NEO_KHZ800);



void CheckPositionLeft(){
  encoder_left.tick();
  watchdog_left = 0;
  tick_duration_left = (millis() - old_tick_duration_left);
  old_tick_duration_left = millis();
  tick_output_left += 1/((tick_duration_left * 333)/1000/60);
  if(tick_counter_left == 4){
    rpm_left = tick_output_left / 5;
    tick_counter_left = 0;
    tick_output_left = 0;
    }
  tick_counter_left++;
}

void CheckPositionRight(){
  encoder_right.tick();
  watchdog_right = 0;
  tick_duration_right = (millis() - old_tick_duration_right);
  old_tick_duration_right = millis();
  tick_output_right += 1/((tick_duration_right * 333)/1000/60);
  if(tick_counter_right == 4){
    rpm_right = tick_output_right / 5;
    tick_counter_right = 0;
    tick_output_right = 0;
    }
  tick_counter_right++;
}

//void CheckPositionLeft(){
//  encoder_left.tick();
//  tick_duration_left = (millis() - old_tick_duration_left);
//  old_tick_duration_left = millis();
//  rpm_left = 1/((tick_duration_left * 333)/1000/60);
//}
//
//void CheckPositionRight(){
//  encoder_right.tick();
//  tick_duration_right = (millis() - old_tick_duration_right);
//  old_tick_duration_right = millis();
//  rpm_right = 1/((tick_duration_right * 333)/1000/60);
//}

//double calculate_rpm_left(){
//  long new_position_left = encoder_left.getPosition();
//  long position_change_left;
//  double RPM_left;
//
//  if (new_position_left != old_position_left) {
//    tick_time_left = (micros() - last_time_left);
//    position_change_left = old_position_left - new_position_left;
//    RPM_left = 1 / ((double(tick_time_left / position_change_left) * 166)/1000000/60); //125?
//    old_position_left = new_position_left;
//    last_time_left = micros();   
//  }
//  //else{
//    //RPM_left = 0.0;
//  //}
//  delay(50);
//  return RPM_left;
//}
//
//double calculate_rpm_right(){
//  long new_position_right = encoder_right.getPosition();
//  long position_change_right;
//  double RPM_right;
//
//  if (new_position_right != old_position_right) {
//    tick_time_right = (micros() - last_time_right);
//    position_change_right = old_position_right - new_position_right;
//    RPM_right = 1 / ((double(tick_time_right / position_change_right) * 166)/1000000/60); // right encoder seems to give 166 ticks per rev....
//    old_position_right = new_position_right;
//    last_time_right = micros();   
//  }
//  //else{
//    //RPM_right = 0.0;
//  //}
//  delay(50);
//  return RPM_right;
//}



double convert_vel_rpm(double vel){
  double RPM; 
  RPM = (vel / 0.19478) * 60;
  return RPM;
}

void battery_status(){
  int scaled;
  float ana1;
  ana1 = (float(analogRead(AnPin1))*(3.3/1023.0) * 175550) / 32450;
  Serial.println(ana1);
  scaled = (ana1 - 12) / (4.8 / 255); // 4.8v range
  if(scaled < 0){
    scaled = 0;
  }
  Serial.println(scaled);
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color((255-scaled), scaled, 0)); //RGB values
  pixels.show();   // Send the updated pixel colors to the hardware.
}



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
  PID_left.SetSampleTime(50);
  PID_left.SetMode(AUTOMATIC);
  PID_left.SetOutputLimits(0,255);
  PID_right.SetSampleTime(50);
  PID_right.SetMode(AUTOMATIC);
  PID_right.SetOutputLimits(0,255);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)

  
} // setup()


// Read the current position of the encoder and print out when changed.
void loop(){
  double vel_r, vel_l, th, x;
  double robot_width = 0.2;

  x = 0.0;//0.5;//-0.8;
  th = 0.1;//0.95;

  while(1){

    vel_r = ((2*x) + (th*robot_width))/2;
    vel_l = ((2*x) - (th*robot_width))/2;
    
    if(vel_l > 0.0){
      SetpointLeft = convert_vel_rpm(vel_l);    
    }
    else{
      SetpointLeft = convert_vel_rpm(-vel_l);
    }
    if(vel_r > 0.0){
      SetpointRight = convert_vel_rpm(vel_r);    
    }
    else{
      SetpointRight = convert_vel_rpm(-vel_r);
    }
    
    
    if (watchdog_left > 20){
      rpm_left = 0;
    }
    if (watchdog_right > 20){
      rpm_right = 0;
    }
    
    InputLeft = rpm_left;//calculate_rpm_left();
    InputRight = rpm_right;//calculate_rpm_right();
    PID_left.Compute();
    PID_right.Compute();
    



    if (vel_l > 0.0){
      left.forwards(char(OutputLeft)); 
    }
    else{
      left.backwards(char(OutputLeft));
    }

    if (vel_r > 0.0){
      right.forwards(char(OutputRight)); 
    }
    else{
      right.backwards(char(OutputRight));
    }
      //Serial.print("Output_PWM:");
      //Serial.print(OutputLeft);
      //Serial.print(",");
      //Serial.print("Input_RPM:");
      //Serial.print(InputLeft);
      //Serial.print(",");
      //Serial.print("Setpoint:");
      //Serial.println(SetpointLeft);
      //Serial.print("tick_output_left:");
      //Serial.print(tick_output_left);
      //Serial.print(",");
      //Serial.print("tick_output_right:");
      //Serial.println(tick_output_right);
      watchdog_left++;
      watchdog_right++;
      battery_status();
  }

       
} // loop ()
