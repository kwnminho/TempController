
/********************************************************
 * PID Temp controller
 * Use Programming Port
 ********************************************************/

#include <PID_v1.h>
#include <stdio.h> // for function sprintf
#include "RunningAverage.h"

#define PIN_INPUT0 A0
#define PIN_INPUT1 A1
#define PIN_INPUT2 A2
#define PIN_INPUT3 A3


//Define Variables we'll be connecting to
double Setpoint[4]={1,1,1,1};
double Input[4];
double Output[4];
double thermvoltage[2];
double thermresistance;
double temperature;
RunningAverage data1(128);
RunningAverage data2(128);
int samples = 0;

//Specify the links and initial tuning parameters
double Kp[4]={0.1,2,2,2};
double Ki[4]={0.1,5,5,5};
double Kd[4]={0.1,1,1,1};
PID myPID0(&Input[0], &Output[0], &Setpoint[0], Kp[0], Ki[0], Kd[0], DIRECT);
//PID myPID1(&Input[1], &Output[1], &Setpoint[1], Kp[1], Ki[1], Kd[1], DIRECT);
//PID myPID2(&Input[2], &Output[2], &Setpoint[2], Kp[2], Ki[2], Kd[2], DIRECT);
//PID myPID3(&Input[3], &Output[3], &Setpoint[3], Kp[3], Ki[3], Kd[3], DIRECT);

void setup()
{
  Serial.begin(115200);
  data1.clear();
  data2.clear();
  // Set ADC to 12bit mode
  analogReadResolution(12); 
  analogWriteResolution(12); 
  //initialize the variables we're linked to
  data1.addValue(analogRead(PIN_INPUT0));
  data2.addValue(analogRead(PIN_INPUT1));
  samples++;
  thermvoltage[0]=data1.getAverage();
  thermvoltage[1]=data2.getAverage();
  double temp=thermvoltage[0]-thermvoltage[1];
  thermresistance=10000*thermvoltage[1]/temp;
  temperature=thermistor(thermresistance);
  Serial.println(temperature);
  //turn the PID on
  myPID0.SetMode(AUTOMATIC);
  myPID0.SetOutputLimits(-3,3);
  myPID0.SetSampleTime(100);
}

void loop()
{
  unsigned long startTime=millis();

  data1.addValue(analogRead(PIN_INPUT0));
  data2.addValue(analogRead(PIN_INPUT1));
  samples++;
  thermvoltage[0]=data1.getAverage();
  thermvoltage[1]=data2.getAverage();
  double temp=thermvoltage[0]-thermvoltage[1];
  thermresistance=10000*thermvoltage[1]/temp;
  temperature=thermistor(thermresistance);
  Input[0] = temperature;
  myPID0.Compute();
  Serial.println(temperature,4);
 // delay(10);
//Serial.println(millis()-startTime);
//Serial.println(t);
}

double thermistor(double R)
{
float A=0.98750e-3;
float B=2.56900e-4;
float C=0.00023e-7;
double logR=log(R);
double temperature=(1.0 / (A + B*logR + C*logR*logR*logR) ) -273.15;
return temperature;
}


