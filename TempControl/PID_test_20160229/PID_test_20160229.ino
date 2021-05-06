
/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
#include <stdio.h> // for function sprintf

#define PIN_INPUT0 A0
#define PIN_INPUT1 A1
#define PIN_INPUT2 A2
#define PIN_INPUT3 A3


//Define Variables we'll be connecting to
double Setpoint[4]={1,1,1,1};
double Input[4];
double Output[4];
int thermvoltage[2];
double thermresistance;
double temperature;

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
  Serial.begin(9600);
  
  // Set ADC to 12bit mode
  analogReadResolution(12); 
  analogWriteResolution(12); 
  //initialize the variables we're linked to
  thermvoltage[0]=analogRead(PIN_INPUT0);
  thermvoltage[1]=analogRead(PIN_INPUT1);
  thermresistance=10000*thermvoltage[1]/thermvoltage[0];
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

  thermvoltage[0]=analogRead(PIN_INPUT0);
  thermvoltage[1]=analogRead(PIN_INPUT1);
  double temp=thermvoltage[0]-thermvoltage[1];
  thermresistance=10000*double(thermvoltage[1]/temp);
  temperature=thermistor(thermresistance);
  Input[0] = temperature;
  myPID0.Compute();
  Serial.println(temperature);
//Serial.println(millis()-startTime);
//double t=thermistor(10512.12);
//Serial.println(t);
delay(100);
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


