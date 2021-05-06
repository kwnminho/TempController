
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
#include "fonts\Arial_bold_14.h"
#include "dollar.h"
#include <Wire.h>      // this is needed for FT6206
#include <Adafruit_FT6206.h> // Touch screeb library

#define TFT_RST 8
#define TFT_DC 9
#define TFT_CS 10

ILI9341_due tft = ILI9341_due(TFT_CS, TFT_DC, TFT_RST);

// PID control libraries
#include <PID_v1.h> // 

// Arithmetric
#include "RunningAverage.h"

#include <DueFlashStorage.h> // non-volatile memory access
DueFlashStorage dueFlashStorage;

#define PIN_INPUT0 A8
#define PIN_INPUT1 A9

//Define Variables we'll be connecting to
double Setpoint = 18;
double Input;
double Output;
double thermvoltage[2];
double thermresistance;
double temperature;
RunningAverage data1(1000); // Put number of samples in the parenthesis
RunningAverage data2(1000);
int samples = 0;

//Specify the links and initial tuning parameters
double Kp = 2;
double Ki = 0.2;
double Kd = 0;
PID myPID0(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
  unsigned long clocktime=millis();
void setup()
{
  Serial.begin(115200);
  //adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  //REG_ADC_MR = (REG_ADC_MR & 0xFFF0FFFF) | 0x00020000;
  // Screen initialization
  tft.begin();
#if SPI_MODE_NORMAL
  char mode[] = "SPI_MODE_NORMAL";
#elif SPI_MODE_EXTENDED
  char mode[] = "SPI_MODE_EXTENDED";
#elif SPI_MODE_DMA
  char mode[] = "SPI_MODE_DMA";
#endif
  tft.printAligned(mode, gTextAlignMiddleCenter);
  tft.setRotation(iliRotation270);
  tft.fillScreen(ILI9341_BLACK);
  tft.drawImage(dollar, 160, 90, dollarWidth, dollarHeight);
  tft.setFont(Arial_bold_14);
  tft.setTextLetterSpacing(2);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextScale(2);

  // ADC initialization
  data1.clear();
  data2.clear();
  // Set ADC to 12bit mode
  analogReadResolution(12);
  analogWriteResolution(12);
  //initialize the variables we're linked to
  //analogWrite(DAC0, 4095);
  analogWrite(DAC1, 2048);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  int dacvoltage=0;
  myPID0.SetOutputLimits(-3, 3);
  myPID0.SetSampleTime(100);
  myPID0.SetMode(AUTOMATIC);

}

void loop()
{
  digitalWrite(4,HIGH);
  unsigned long startTime = millis();
  int count = 0;
  int dactemp;
  int dacvoltage;
  while (millis() - startTime < 100)
  {
    data1.addValue(analogRead(PIN_INPUT0));
    data2.addValue(analogRead(PIN_INPUT1));
    samples++;
    count++;

  }

  thermvoltage[0] = data1.getAverage();
  thermvoltage[1] = data2.getAverage();
  double temp = thermvoltage[0] - thermvoltage[1];
  thermresistance = 9820 * thermvoltage[1] / temp;
  temperature = thermistor(thermresistance);
  dactemp=abs(int((temperature-15.0)*410.0))%4095;
  Input = temperature;
  myPID0.Compute();
  //Serial.println(dactemp);
  //Serial.println(Output);
  if (millis()-clocktime>1000)
  {
  Serial.println(temperature, 4);
  clocktime=millis();
  }
  dacvoltage=int((Output+3.0)*682);
  analogWrite(DAC0, dacvoltage);
  analogWrite(DAC1,dactemp);
  // tft.setTextArea(30, 30, 26, 12, Arial_bold_14);
  tft.cursorToXY(20, 10); tft.setTextScale(2);
  tft.print(F("Tset= "));
  tft.print(Setpoint, 3);
  tft.print(F(" C "));

  tft.cursorToXY(20, 40);
  tft.print(F("Tsen= "));
  tft.print(temperature, 3);
  tft.print(F(" C "));

  tft.cursorToXY(20, 70);
  tft.print(F("R= "));
  tft.print(thermresistance, 0);
  tft.print(F(" Ohm "));

  tft.cursorToXY(20, 110);
  tft.print(F("DAC TEMP= "));
  tft.print(dactemp);
  
  tft.cursorToXY(20, 140);
  tft.print(F("V= "));
  tft.print(Output,3);
  tft.print(F(" V "));
  
//  tft.cursorToXY(20, 100);
//  tft.print(F("V= "));
//  tft.print(float(random(100) / 100.0), 2);
//  tft.print(F(" V "));
//
//  tft.cursorToXY(20, 130);
//  tft.print(F("I= "));
//  tft.print(float(random(100) / 100.0), 2);
//  tft.print(F(" A "));

  tft.cursorToXY(20, 200); tft.setTextScale(1);
  tft.print(F("Refresh time:"));
  tft.print(millis() - startTime);
  tft.print(F(" ms "));
  //Serial.println(millis()-startTime);
  //Serial.println(t);
}

double thermistor(double R)
{
  float A = 0.98750e-3;
  float B = 2.56900e-4;
  float C = 0.00023e-7;
  double logR = log(R);
  double temperature = (1.0 / (A + B * logR + C * logR * logR * logR) ) - 273.15;
  return temperature;
}


