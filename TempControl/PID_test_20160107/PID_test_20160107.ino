/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
#include <stdio.h> // for function sprintf

#define PIN_INPUT0 A0
#define PIN_INPUT1 A2
#define PIN_INPUT2 A3
#define PIN_INPUT3 A4


//Define Variables we'll be connecting to
double Setpoint[4]={1,1,1,1};
double Input[4];
double Output[4];

//Specify the links and initial tuning parameters
double Kp[4]={0.1,2,2,2};
double Ki[4]={0.1,5,5,5};
double Kd[4]={0.1,1,1,1};
PID myPID0(&Input[0], &Output[0], &Setpoint[0], Kp[0], Ki[0], Kd[0], DIRECT);
PID myPID1(&Input[1], &Output[1], &Setpoint[1], Kp[1], Ki[1], Kd[1], DIRECT);
PID myPID2(&Input[2], &Output[2], &Setpoint[2], Kp[2], Ki[2], Kd[2], DIRECT);
PID myPID3(&Input[3], &Output[3], &Setpoint[3], Kp[3], Ki[3], Kd[3], DIRECT);

void setup()
{
  Serial.begin(9600);
  
  // Set ADC to 12bit mode
  analogReadResolution(12); 
  //initialize the variables we're linked to
  Input[0] = 3.3/4096*analogRead(PIN_INPUT0);
  Input[1] = 3.3/4096*analogRead(PIN_INPUT1);
  Input[2] = 3.3/4096*analogRead(PIN_INPUT2);
  Input[3] = 3.3/4096*analogRead(PIN_INPUT3);

  //turn the PID on
  myPID0.SetMode(AUTOMATIC);
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  myPID3.SetMode(AUTOMATIC);
  
  myPID0.SetOutputLimits(-3,3);
  myPID1.SetOutputLimits(0,3);
  myPID2.SetOutputLimits(0,3);
  myPID3.SetOutputLimits(0,3);
  
  myPID0.SetSampleTime(100);
  myPID1.SetSampleTime(100);
  myPID2.SetSampleTime(100);
  myPID3.SetSampleTime(100);
  
}

void loop()
{
  unsigned long startTime=millis();
  for(int i=0;i<1000;i++)
  {
  Input[0] = 3.3/4096*analogRead(PIN_INPUT0);
  Input[1] = 3.3/4096*analogRead(PIN_INPUT1);
  Input[2] = 3.3/4096*analogRead(PIN_INPUT2);
  Input[3] = 3.3/4096*analogRead(PIN_INPUT3);
myPID0.Compute();
  myPID1.Compute();
  myPID2.Compute();
  myPID3.Compute();
  }
Serial.println(millis()-startTime);
}


