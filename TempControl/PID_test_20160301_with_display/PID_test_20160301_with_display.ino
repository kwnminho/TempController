
/********************************************************
 * Temperature controller
 * To be worked with voltage controlled TEC driver(ADN8843) and
 * Adafruit 2.8" touchscreen or ILI9341 library compatible screen.
 * Use Programming Port
 ********************************************************/
//General Libraries
#include <stdio.h> // for function sprintf

// Display Libraries
#include <SPI.h>
#include <ILI9341_due_config.h>
#include <ILI9341_due.h>
#include "fonts\SystemFont5x7.h"
#include "fonts\Arial_bold_14.h"

#define TFT_RST 8
#define TFT_DC 9
#define TFT_CS 10

ILI9341_due tft = ILI9341_due(TFT_CS, TFT_DC, TFT_RST);

// PID control libraries
#include <PID_v1.h> // 

// Arithmetric
#include "RunningAverage.h"

#define PIN_INPUT0 A8
#define PIN_INPUT1 A9


//Define Variables we'll be connecting to
double Setpoint=1;
double Input;
double Output;
double thermvoltage[2];
double thermresistance;
double temperature;
RunningAverage data1(128); // Put number of samples in the parenthesis
RunningAverage data2(128);
int samples = 0;

//Specify the links and initial tuning parameters
double Kp=0.1;
double Ki=0.1;
double Kd=0.1;
PID myPID0(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup()
{
  Serial.begin(115200);
  while (!Serial) ; 
  // Touch screen initialization
  tft.begin();

  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft.readcommand8(ILI9341_RDMODE);
  Serial.print(F("Display Power Mode: 0x")); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDMADCTL);
  Serial.print(F("MADCTL Mode: 0x")); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDPIXFMT);
  Serial.print(F("Pixel Format: 0x")); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDIMGFMT);
  Serial.print(F("Image Format: 0x")); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDSELFDIAG);
  Serial.print(F("Self Diagnostic: 0x")); Serial.println(x, HEX); 
  
  // ADC initialization
  data1.clear();
  data2.clear();
  // Set ADC to 12bit mode
  analogReadResolution(12); 
  analogWriteResolution(12); 
  //initialize the variables we're linked to
  analogWrite(DAC0,4095);
  analogWrite(DAC1,2048);
//  data1.addValue(analogRead(PIN_INPUT0));
//  data2.addValue(analogRead(PIN_INPUT1));
//  samples++;
//  thermvoltage[0]=data1.getAverage();
//  thermvoltage[1]=data2.getAverage();
//  double temp=thermvoltage[0]-thermvoltage[1];
//  thermresistance=10000*thermvoltage[1]/temp;
//  temperature=thermistor(thermresistance);
//  Serial.println(temperature);
//  //turn the PID on
//  myPID0.SetMode(AUTOMATIC);
//  myPID0.SetOutputLimits(-3,3);
//  myPID0.SetSampleTime(100);
}

void loop()
{
  unsigned long startTime=millis();
  analogWrite(DAC0,4095);
  analogWrite(DAC1,2048);
  data1.addValue(analogRead(PIN_INPUT0));
  data2.addValue(analogRead(PIN_INPUT1));
  samples++;
  thermvoltage[0]=data1.getAverage();
  thermvoltage[1]=data2.getAverage();
  double temp=thermvoltage[0]-thermvoltage[1];
  thermresistance=10000*thermvoltage[1]/temp;
  temperature=thermistor(thermresistance);
  Input= temperature;
  myPID0.Compute();
  Serial.println(temperature,4);
  testFillScreen();
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

unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(ILI9341_BLACK);
  tft.fillScreen(ILI9341_RED);
//  tft.fillScreen(ILI9341_GREEN);
//  tft.fillScreen(ILI9341_BLUE);
//  tft.fillScreen(ILI9341_BLACK);
  return micros() - start;
}

