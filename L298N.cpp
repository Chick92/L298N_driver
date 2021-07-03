
#include "Arduino.h"
#include "L298N.h"
#include <RotaryEncoder.h>
#include <PID_v1.h>

L298N::L298N(char IN1, char IN2, char EN, char INT1, char INT2)
{
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(EN, OUTPUT);

	_INT1 = INT1;
	_INT2 = INT2;
	attachInterrupt(digitalPinToInterrupt(_INT1), checkPosition, CHANGE);
	attachInterrupt(digitalPinToInterrupt(_INT2), checkPosition, CHANGE);
	pinMode(INT1, INPUT);
	pinMode(INT2, INPUT);
	
	last_time = millis();
	tick_time = 0;
	old_position = 0;
	
	Setpoint = 0;
	PID.SetSampleTime(20);
	PID.SetMode(AUTOMATIC);
	PID.SetOutputLimits(0,255);
	
	_IN1 = IN1;
	_IN2 = IN2;
	_EN = EN;

	
}

void L298N::checkPosition(){
  encoder.tick(); // just call tick() to check the state.
}

double L298N::calculate_rpm(){
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
	delay(20); // required for dagu as encoders are shit and only pulse 125 times per rev
	return RPM;
}

void L298N::set_rpm(int rpm){
	Setpoint = rpm;//covert_vel_rpm(vel);
	Input = calculate_rpm();
	myPID.Compute();

	if (Setpoint > 0.0){
		forwards(char(Output)); 
	}
	else{
		backwards(char(Output));
	}	
}

void L298N::forwards(char pwm){
  digitalWrite(_IN1, HIGH); 
  digitalWrite(_IN2, LOW); 
  analogWrite(_EN, pwm);
  return; 
}

void L298N::backwards(char pwm){
  digitalWrite(_IN1, LOW); 
  digitalWrite(_IN2, HIGH); 
  analogWrite(_EN, pwm);
  return;
}