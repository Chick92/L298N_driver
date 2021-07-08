#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <Arduino.h>
#include <RotaryEncoder.h>
#include <L298N.h>

char A_IN1 = 8, A_IN2 = 9, A_EN = 17, A_INT1 = 4, A_INT2 = 5;

byte ATuneModeRemember=2;
double input=80, output=50, setpoint=180;
double kp=2,ki=0.5,kd=2;

double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;

boolean tuning = true;
unsigned long  modelTime, serialTime;


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
L298N left(A_IN1, A_IN2, A_EN);


PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output);

//set to false to connect to the real world
boolean useSimulation = false;

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

double convert_vel_rpm(double vel){
  double RPM; 
  RPM = (vel / 0.19478) * 60;
  return RPM;
}

void setup()
{
  attachInterrupt(digitalPinToInterrupt(A_INT1), CheckPositionLeft, CHANGE);
  pinMode(A_INT1, INPUT);
  attachInterrupt(digitalPinToInterrupt(A_INT2), CheckPositionLeft, CHANGE);
  pinMode(A_INT2, INPUT);


  if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=outputStart;
    }
    modelTime = 0;
  }
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;
  Serial.begin(9600);

}

void loop()
{

  unsigned long now = millis();
  //if (watchdog_left > 100){
  //    rpm_left = 0;
  //  }

  if(!useSimulation)
  { //pull the input in from the real world
    input = rpm_left;
  }
  
  if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else myPID.Compute();
  
  if(useSimulation)
  {
    theta[30]=output;
    if(now>=modelTime)
    {
      modelTime +=100; 
      DoModel();
    }
  }
  else
  {
     left.forwards(char(output)); 
  }
  
  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
  

    watchdog_left++;
}

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}


void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}

void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the input
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;

}
