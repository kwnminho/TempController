
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
#include "fonts\Arial14.h"
#include "dollar.h" // Saffman lab logo.
#include <Wire.h>      // this is needed for FT6206
#include <Adafruit_FT6206.h> // Touch screeb library

#define TFT_RST 8
#define TFT_DC 9
#define TFT_CS 10

ILI9341_due tft = ILI9341_due(TFT_CS, TFT_DC, TFT_RST);
Adafruit_FT6206 ctp = Adafruit_FT6206(); // The FT6206 uses hardware I2C (SCL/SDA)

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
    ctp.begin();
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
  tft.setFont(Arial14);
  tft.setTextLetterSpacing(1);
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
  labeldrawer();
}

void loop()
{
  unsigned long startTime = millis();
  int count = 0;
  int dactemp;
  int dacvoltage;

  while (millis() - startTime < 80)
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
  // fake value
  thermresistance=9900+samples/200;
  //
  temperature = thermistor(thermresistance);
  dactemp=abs(int((temperature-15.0)*410.0))%4095;
  Input = temperature;
  myPID0.Compute();
  //Serial.println(dactemp);
  //Serial.println(Output);
  if (millis()-clocktime>1000)
  {
  Serial.println((int)temperature);
  clocktime=millis();
  }
  dacvoltage=int((Output+3.0)*682);
  analogWrite(DAC0, dacvoltage);
  analogWrite(DAC1,dactemp);
  //tft.fillScreen(ILI9341_BLACK);
  //tft.setRotation(0);
  
  //Left half

draw7Number((int)Setpoint,60,20,2,ILI9341_WHITE,ILI9341_BLACK,2);
draw7Number(round(1000*Setpoint)%1000,110,35,1,ILI9341_WHITE,ILI9341_BLACK,3);


draw7Number(floor(temperature),60,20+55,2,ILI9341_WHITE,ILI9341_BLACK,2);
draw7Number(abs((int)1000*temperature-1000*(int)temperature),110,35+55,1,ILI9341_WHITE,ILI9341_BLACK,3);


draw7Number(floor(temperature),60,20+55+55,2,ILI9341_WHITE,ILI9341_BLACK,2);
draw7Number(abs((int)1000*temperature-1000*(int)temperature),110,35+55+55,1,ILI9341_WHITE,ILI9341_BLACK,3);

//Right Half
float fakevoltage=random(-300,300)/100.0;
float fakecurrent=random(-150,150)/100.0;

  
draw7Number(int(fakevoltage),60+150,20,2,ILI9341_WHITE,ILI9341_BLACK,2);
draw7Number(abs((int)100*fakevoltage-100*(int)fakevoltage),110+150,35,1,ILI9341_WHITE,ILI9341_BLACK,2);


draw7Number(floor(fakecurrent),60+150,20+55,2,ILI9341_WHITE,ILI9341_BLACK,1);
draw7Number(abs((int)100*fakecurrent-100*(int)fakecurrent),110+150,35+55,1,ILI9341_WHITE,ILI9341_BLACK,2);

//tft.setTextArea(30, 30, 26, 12, Arial_bold_14);
  
//  tft.cursorToXY(20, 10); tft.setTextScale(2);
//  tft.print(samples, 3);

//
//  tft.cursorToXY(20, 40);
//  tft.print(F("Tsen= "));
//  tft.print(temperature, 3);
//  tft.print(F(" C "));
//
//  tft.cursorToXY(20, 70);
//  tft.print(F("R= "));
//  tft.print(thermresistance, 0);
//  tft.print(F(" Ohm "));
//
//  tft.cursorToXY(20, 110);
//  tft.print(F("DAC TEMP= "));
//  tft.print(dactemp);
//  
//  tft.cursorToXY(20, 140);
//  tft.print(F("V= "));
//  tft.print(Output,3);
//  tft.print(F(" V "));
//  
  tft.cursorToXY(20, 200); tft.setTextScale(1);
  tft.print(F("Refresh time:"));
  tft.print(millis() - startTime);
  tft.print(F(" ms "));

  if (ctp.touched())
  {
    
  }
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

void draw7Number(int n, unsigned int xLoc, unsigned int yLoc, char cS, unsigned int fC, unsigned int bC, char nD) {
  unsigned int num = abs(n), i, s, t, w, col, h, a, b, si = 0, j = 1, d = 0;
  unsigned int S2 = 5 * cS;  // width of horizontal segments   5 times the cS
  unsigned int S3 = 2 * cS;  // thickness of a segment 2 times the cs
  unsigned int S4 = 7 * cS;  // height of vertical segments 7 times the cS
  unsigned int x1 = cS + 1;  // starting x location of horizontal segments
  unsigned int x2 = S3 + S2 + 1; // starting x location of right side segments
  unsigned int y1 = yLoc + x1; // starting y location of top side segments
  unsigned int y3 = yLoc + S3 + S4 + 1; // starting y location of bottom side segments
  unsigned int seg[7][3] = {{x1, yLoc, 1}, {x2, y1, 0}, {x2, y3 + x1, 0}, {x1, (2 * y3) - yLoc, 1}, {0, y3 + x1, 0}, {0, y1, 0}, {x1, y3, 1}}; // actual x,y locations of all 7 segments with direction
  unsigned char nums[12] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x67, 0x00, 0x40};  // segment defintions for all 10 numbers plus blank and minus sign
  unsigned char c, cnt;

  c = abs(cS);         // get character size between 1 and 10 ignoring sign
  if (c > 10) c = 10;
  if (c < 1) c = 1;

  cnt = abs(nD);      // get number of digits between 1 and 10 ignoring sign
  if (cnt > 10) cnt = 10;
  if (cnt < 1) cnt = 1;

  xLoc += (cnt - 1) * (d = S2 + (3 * S3) + 2); // set starting x at last digit location

  while ( cnt > 0) {                  // for cnt number of places

    --cnt;

    if (num > 9) i = num % 10;         //get the last digit
    else if (!cnt && n < 0) i = 11;            //show minus sign if 1st position and negative number
    else if (nD < 0 && !num && abs(nD) - cnt > 1) i = 10; //show blanks if remaining number is zero
    else i = num;

    num = num / 10; // trim this digit from the number

    for (j = 0; j < 7; ++j) {          // draw all seven segments

      if (nums[i] & (1 << j)) col = fC;  // if segment is On use foreground color
      else col = bC;                    // else use background color

      if (seg[j][2]) {

        w = S2;                        // Starting width of segment (side)
        t = seg[j][1] + S3;            // maximum thickness of segment
        h = seg[j][1] + cS;            // half way point thickness of segment
        a = xLoc + seg[j][0] + cS;     // starting x location
        b = seg[j][1];                 // starting y location

        while (b < h) {                // until x location = half way
          tft.drawFastHLine(a, b, w, col); //Draw a horizontal segment top
          a--;                         // move the x position by -1
          b++;                         // move the y position by 1
          w += 2;                      // make the line wider by 2
        }

      } else {

        w = S4;                        // Starting height of segment (side)
        t = xLoc + seg[j][0] + S3;     // maximum thickness of segment
        h = xLoc + seg[j][0] + cS;     // half way point thickness of segment
        a = seg[j][1] + cS;            // starting y location
        b = xLoc + seg[j][0];          // starting x location

        while (b < h) {                // until x location = half way
          tft.drawFastVLine(b, a, w, col);  // Draw a vertical line right side
          a--;                          //  move the y position by -1
          b++;                          //  move teh x position by 1
          w += 2;                       //  make the line wider by 2
        }
      }

      while (b < t) {  //finish drawing horizontal bottom or vertical left side of segment
        if (seg[j][2]) {
          tft.drawFastHLine(a, b, w, col);  // Draw Horizonal line bottom
        } else {
          tft.drawFastVLine(b, a, w, col);  // Draw Vertical line left side
        }
        b++;        // keep moving the x or y draw position until t
        a++;        // move the length or height starting position back the other way.
        w -= 2;     // move the length or height back the other way
      }
    }

    xLoc -= d;      // move to next digit position
  }
}

void labeldrawer(void)
{
  
  tft.cursorToXY(5, 25); tft.setTextScale(3);
  tft.print(F("T"));
  tft.cursorToXY(25, 45); tft.setTextScale(1);
  tft.print(F("set"));
  tft.cursorToXY(50, 25); tft.setTextScale(3);
  tft.print(F(":"));
  tft.cursorToXY(150, 45); tft.setTextScale(1);
  tft.print(F(" C "));
  tft.cursorToXY(5, 25+55); tft.setTextScale(3);
  tft.print(F("T"));
  tft.cursorToXY(25, 45+55); tft.setTextScale(1);
  tft.print(F("sen"));
  tft.cursorToXY(50, 25+55); tft.setTextScale(3);
  tft.print(F(":"));
  tft.cursorToXY(150, 45+55); tft.setTextScale(1);
  tft.print(F(" C "));
  tft.cursorToXY(5, 25+55+55); tft.setTextScale(3);
  tft.print(F("T"));
  tft.cursorToXY(25, 45+55+55); tft.setTextScale(1);
  tft.print(F("sink"));
  tft.cursorToXY(50, 25+55+55); tft.setTextScale(3);
  tft.print(F(":"));
  tft.cursorToXY(150, 45+55+55); tft.setTextScale(1);
  tft.print(F(" C "));
  tft.cursorToXY(5+10+150, 25); tft.setTextScale(3);
  tft.print(F("V"));
  tft.cursorToXY(50+150, 25); tft.setTextScale(3);
  tft.print(F(":"));
  tft.cursorToXY(150+150-5, 45); tft.setTextScale(1);
  tft.print(F(" V "));
  
    tft.cursorToXY(5+25+150, 25+55); tft.setTextScale(3);
  tft.print(F("I"));
  tft.cursorToXY(50+150, 25+55); tft.setTextScale(3);
  tft.print(F(":"));
  tft.cursorToXY(150+150-5, 45+55); tft.setTextScale(1);
  tft.print(F(" A "));
}
