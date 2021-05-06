
/********************************************************
   Temperature controller
   To be worked with voltage controlled TEC driver(ADN8843) and
   Adafruit 2.8" touchscreen or ILI9341 library compatible screen.
   Use Programming Port
 ********************************************************/
//General Libraries
#include <stdio.h> // for function sprintf
#include "AH_MCP320x.h" // ADC Library
// Display Libraries
#include <SPI.h>
#include <ILI9341_due_config.h>
#include <ILI9341_due.h>
#include "fonts\Arial14.h"
//#include "dollar.h" // Saffman lab logo.
#include <Wire.h>      // this is needed for FT6206
#include <Adafruit_FT6206.h> // Touch screeb library

#define ADC_CS 53
#define ADC2_CS 2
#define LDAC2 6
#define TFT_RST 8
#define TFT_DC 9
#define TFT_CS 10

#define ADC_PIN_VTEC 0
#define ADC_PIN_ITEC 1
#define ADC_PIN_TH1REF 2
#define ADC_PIN_TH1POS 3
#define ADC_PIN_TH2REF 4
#define ADC_PIN_TH2POS 5
#define ADC_PIN_TH_INT 7

#define DAC_PIN_VCON 0
#define DAC_PIN_TMON 1
#define DAC_PIN_DACC 2

ILI9341_due tft = ILI9341_due(TFT_CS, TFT_DC, TFT_RST);
Adafruit_FT6206 ctp = Adafruit_FT6206(); // The FT6206 uses hardware I2C (SCL/SDA)

// PID control libraries
#include <PID_v1.h> // 

// Arithmetric
#include "RunningAverage.h"

//MCP4728 Eval
#include "mcp4728.h"
mcp4728 dac = mcp4728(0); // instantiate mcp4728 object, Device ID = 0
mcp4728 dac2 = mcp4728(1); // instantiate mcp4728 object, Device ID = 7


#include <DueFlashStorage.h> // non-volatile memory access
DueFlashStorage dueFlashStorage;


//Define Variables
double Setpoint = 18;
double Input, Output;
double thermvoltage[4];
double thermresistance, thermresistance2;
double temperature, temperature2;
float Vsense, Isense, VTEC, ITEC;
int select = 0;
RunningAverage data1(300); // Put number of samples in the parenthesis
RunningAverage data2(300);
RunningAverage data1_sink(30);
RunningAverage data2_sink(30);
RunningAverage dataVTEC(20);
RunningAverage dataITEC(20);
int samples = 0;
int screen = 0, redraw = 0;
//Specify the links and initial tuning parameters
double Kp = 2.5;
double Ki = 0.1;
double Kd = 0;
PID myPID0(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
unsigned long clocktime = millis();

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
  tft.setRotation(iliRotation90);
  tft.fillScreen(ILI9341_BLACK);
  //tft.drawImage(dollar, 180, 130, dollarWidth, dollarHeight);
  tft.setFont(Arial14);
  tft.setTextLetterSpacing(1);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextScale(2);
  //MCP4728 init
  dac.begin();  // initialize i2c interface
  dac.vdd(3277); // set VDD(mV) of MCP4728 for correct conversion between LSB and Vout
  dac.setVref(0, 0, 0, 0);  // Use internal voltage reference (2.048V)
  dac.setGain(0, 0, 0, 0);  // Set the gain of internal voltage reference ( 0 = gain x1, 1 = gain x2 )
  dac.setPowerDown(0, 0, 0, 0); // 1 = 1K ohms to GND, 2 = 100K ohms to GND, 3 = 500K ohms to GND)

  dac2.begin();  // initialize i2c interface
  dac2.vdd(3277); // set VDD(mV) of MCP4728 for correct conversion between LSB and Vout
  dac2.setVref(0, 0, 0, 0);  // Use internal voltage reference (2.048V)
  dac2.setGain(0, 0, 0, 0);  // Set the gain of internal voltage reference ( 0 = gain x1, 1 = gain x2 )
  dac2.setPowerDown(0, 0, 0, 0); // 1 = 1K ohms to GND, 2 = 100K ohms to GND, 3 = 500K ohms to GND)

  
  pinMode(LDAC2,OUTPUT);
  digitalWrite(LDAC2, LOW);
  printStatus(); // Print all internal value and setting of input register and EEPROM.
  printStatus2();

  // ADC initialization
  data1.clear();
  data2.clear();
  data1_sink.clear();
  data2_sink.clear();
  dataVTEC.clear();
  dataITEC.clear();

  //initialize the variables we're linked to

  myPID0.SetOutputLimits(-4.9, 4.9);
  myPID0.SetSampleTime(50);
  myPID0.SetMode(AUTOMATIC);

  pinMode(31,OUTPUT);
  pinMode(33,INPUT_PULLUP);
  digitalWrite(31,LOW);
  labeldrawer();

}

void loop()
{

  unsigned long startTime = millis();

  int count = 0;
  int dactemp;
  int num = samples;
  int ADCvalue;
  AH_MCP320x ADC_SPI(ADC_CS);
  AH_MCP320x ADC2_SPI(ADC2_CS);
  //dac.analogWrite(DAC_PIN_TMON,4095);
 // dac.voutWrite(DAC_PIN_TMON,1250);
  while (millis() - startTime < 40)
  {
    ADCvalue = ADC_SPI.readCH(ADC_PIN_TH1REF);
    data1.addValue(ADCvalue);
    ADCvalue = ADC_SPI.readCH(ADC_PIN_TH1POS);
    data2.addValue(ADCvalue);
    samples++;
    count++;
  }
  Serial.println(count);
  while (millis() - startTime < 42)
  {
    ADCvalue = ADC_SPI.readCH(ADC_PIN_VTEC);    
    dataVTEC.addValue(ADCvalue);
    ADCvalue = ADC_SPI.readCH(ADC_PIN_ITEC);
    dataITEC.addValue(ADCvalue);
  }
  while (millis() - startTime < 44)
  {
    //ADCvalue = ADC_SPI.readCH(ADC_PIN_TH_INT);
    ADCvalue = ADC2_SPI.readCH(ADC_PIN_TH_INT);
    data1_sink.addValue(ADCvalue);
  }
  Vsense = dataVTEC.getAverage();
  Isense = dataITEC.getAverage();
  VTEC = 4.0 * ((3.3 * Vsense / 4096.0) - 1.25);
  ITEC = ((3.3 * Isense / 4096.0) - 1.25) / 0.525;

  thermvoltage[0] = data1.getAverage();
  thermvoltage[1] = data2.getAverage();
  thermvoltage[2] = 4095;
  thermvoltage[3] = data1_sink.getAverage();
  double VRref = thermvoltage[0] - thermvoltage[1];
  double VRref2 = thermvoltage[2] - thermvoltage[3];

  thermresistance = 10000 * thermvoltage[1] / VRref;
  thermresistance2 = 10000 * thermvoltage[3] / VRref2;
  temperature = thermistor10k(thermresistance);
  temperature2 = thermistor10k(thermresistance2);
  //dactemp = abs(int((temperature - 15.0) * 410.0)) % 4095;
  Input = temperature;
  
  int dacvoltage=1250-200*Output;
  dac.voutWrite(DAC_PIN_TMON,dacvoltage);
  dac.voutWrite(DAC_PIN_VCON,dacvoltage);

  if (VRref < 0)
  {
    //    printmessage("No Rref or Rth");
    tft.cursorToXY(140, 220); tft.setTextScale(1);
    tft.print(F("R open"));
  }
  else {
    tft.cursorToXY(140, 220); tft.setTextScale(1);
    tft.print(F("10k Rth"));
  }
  
  if (Input > -20 && Input < 100)
  {
    myPID0.Compute();
    tft.cursorToXY(140, 200); tft.setTextScale(1);
    tft.print(F("PID Enabled "));
  } else
  {
    tft.cursorToXY(140, 200); tft.setTextScale(1);
    tft.print(F("PID Disabled"));
  }
  int activatescreen=digitalRead(33);
  activatescreen=1;
  if (activatescreen ==1)
  {
  //Left half
  if (screen == 0)
  {
    draw7Number((int)Setpoint, 60, 20, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(round(1000 * Setpoint) % 1000, 110, 35, 1, ILI9341_WHITE, ILI9341_BLACK, 3);

    draw7Number(floor(temperature), 60, 20 + 55, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)1000 * temperature - 1000 * (int)temperature), 110, 35 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 3);

    draw7Number(floor(temperature2), 60, 20 + 55 + 55, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)1000 * temperature2 - 1000 * (int)temperature2), 110, 35 + 55 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 3);

    //Right Half

    draw7Number((int)(VTEC * 100.0), 60 + 150, 20, 1, ILI9341_WHITE, ILI9341_BLACK, -4);
    //   draw7Number(abs((int)100 * VTEC - 100 * (int)VTEC), 110 + 150, 35, 1, ILI9341_WHITE, ILI9341_BLACK, 2);

    draw7Number((int)(ITEC * 100.0), 60 + 150, 20 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, -4);
    // draw7Number(abs((int)100 * ITEC - 100 * (int)ITEC), 110 + 150, 35 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 2);
  }
  else if (screen == 1)
  {
    //Left half

    draw7Number((int)Setpoint, 60, 20, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(round(1000 * Setpoint) % 1000, 110, 35, 1, ILI9341_WHITE, ILI9341_BLACK, 3);

    draw7Number(floor(Kp), 60, 20 + 55, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)100 * Kp - 100 * (int)Kp), 110, 35 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 2);

    //Right Half
    draw7Number((int)Ki, 60 + 150, 20, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)100 * Ki - 100 * (int)Ki), 110 + 150, 35, 1, ILI9341_WHITE, ILI9341_BLACK, 2);

    draw7Number((int)Kd, 60 + 150, 20 + 55, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)100 * Kd - 100 * (int)Kd), 110 + 150, 35 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 2);
  }
  tft.cursorToXY(270, 220); tft.setTextScale(1);
  tft.print(millis() - startTime);
  tft.print(F("ms"));
  //  touchaction(ctp.touched());
  touchaction2();
  }
  Serial.println(temperature);
}

double thermistor10k(double R)
{
  float A = 0.98750e-3;
  float B = 2.56900e-4;
  float C = 0.00023e-7;
  double logR = log(R);
  double temperature = (1.0 / (A + B * logR + C * logR * logR * logR) ) - 273.15;
  return temperature;
}

double thermistor50k(double R)
{
  float A = 7.6647e-4;
  float B = 2.3051e-4;
  float C = 7.3815e-8;
  double logR = log(R);
  double temperature = (1.0 / (A + B * logR + C * logR * logR * logR) ) - 273.15;
  return temperature;
}

void draw7Number(int n, unsigned int xLoc, unsigned int yLoc, char cS, unsigned int fC, unsigned int bC, int nD) {
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
    else if (nD < 0 && !num) i = 10; //show blanks if remaining number is zero
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
  tft.cursorToXY(5, 25 + 55); tft.setTextScale(3);
  tft.print(F("T"));
  tft.cursorToXY(25, 45 + 55); tft.setTextScale(1);
  tft.print(F("sen"));
  tft.cursorToXY(50, 25 + 55); tft.setTextScale(3);
  tft.print(F(":"));
  tft.cursorToXY(150, 45 + 55); tft.setTextScale(1);
  tft.print(F(" C "));
  tft.cursorToXY(5, 25 + 55 + 55); tft.setTextScale(3);
  tft.print(F("T"));
  tft.cursorToXY(25, 45 + 55 + 55); tft.setTextScale(1);
  tft.print(F("sink"));
  tft.cursorToXY(50, 25 + 55 + 55); tft.setTextScale(3);
  tft.print(F(":"));
  tft.cursorToXY(150, 45 + 55 + 55); tft.setTextScale(1);
  tft.print(F(" C "));
  tft.cursorToXY(5 + 10 + 150, 25); tft.setTextScale(3);
  tft.print(F("V"));
  tft.cursorToXY(50 + 150, 25); tft.setTextScale(3);
  tft.print(F(":"));
  tft.cursorToXY(150 + 150 - 5, 45); tft.setTextScale(1);
  tft.print(F(" V "));

  tft.cursorToXY(5 + 25 + 150, 25 + 55); tft.setTextScale(3);
  tft.print(F("I"));
  tft.cursorToXY(50 + 150, 25 + 55); tft.setTextScale(3);
  tft.print(F(":"));
  tft.cursorToXY(150 + 150 - 5, 45 + 55); tft.setTextScale(1);
  tft.print(F(" A "));

  tft.cursorToXY(1, 225); tft.setTextScale(1);
  tft.print(F("2016-06-06 Build"));
}

void labeldrawer2(void)
{

  tft.cursorToXY(5, 25); tft.setTextScale(3);
  tft.print(F("T"));
  tft.cursorToXY(25, 45); tft.setTextScale(1);
  tft.print(F("set"));
  tft.cursorToXY(50, 25); tft.setTextScale(3);
  tft.print(F(":"));
  tft.cursorToXY(150, 45); tft.setTextScale(1);
  tft.print(F(" C "));
  tft.cursorToXY(5, 25 + 55); tft.setTextScale(2);
  tft.print(F("Kp"));
  tft.cursorToXY(50, 25 + 55); tft.setTextScale(2);
  tft.print(F(":"));
  tft.cursorToXY(5 + 10 + 150, 25); tft.setTextScale(2);
  tft.print(F("Ki"));
  tft.cursorToXY(50 + 150, 25); tft.setTextScale(2);
  tft.print(F(":"));
  tft.cursorToXY(5 + 25 + 150, 25 + 55); tft.setTextScale(2);
  tft.print(F("Kd"));
  tft.cursorToXY(50 + 150, 25 + 55); tft.setTextScale(2);
  tft.print(F(":"));
  tft.cursorToXY(1, 225); tft.setTextScale(1);
  tft.print(F("Config Screen"));
}

void touchaction(int trigger)
{
  if (trigger)
  {
    tft.cursorToXY(200, 220); tft.setTextScale(1);
    tft.print(F("T"));
  }
  if (!trigger)
  {
    tft.cursorToXY(200, 220); tft.setTextScale(1);
    tft.print(F("N "));
  }
}

void touchaction2(void)
{
  int box[4] = {240, 320, 200, 240};
  int Tsetbox[4] = {0, 160, 0, 50};
  int Pbox[4] = {0, 160, 51, 100};
  int Ibox[4] = {160, 320, 0, 50};
  int Dbox[4] = {160, 320, 51, 100};
  int increasebox[4] = {160, 320, 100, 200};
  int decreasebox[4] = {0, 160, 100, 240};


  if (ctp.touched())  {
    TS_Point p = ctp.getPoint();
    int coord_x = p.y;
    int coord_y = 240 - p.x;
    if (coord_x > box[0] && coord_x < box[1] && coord_y > box[2] && coord_y < box[3]) {
      redraw = 1;
      if (screen == 0)
      {
        screen = 1;
      }
      else if (screen == 1)
      {
        screen = 0;
      }
      screenupdate();
    }

    else if (screen == 1)
    {
      if (coord_x > Tsetbox[0] && coord_x < Tsetbox[1] && coord_y > Tsetbox[2] && coord_y < Tsetbox[3])
      {
        select = 1;
        tft.cursorToXY(140, 200); tft.setTextScale(1);
        tft.print(F("1"));
      }
      else if (coord_x > Pbox[0] && coord_x < Pbox[1] && coord_y > Pbox[2] && coord_y < Pbox[3]) {
        select = 2;
        tft.cursorToXY(140, 200); tft.setTextScale(1);
        tft.print(F("2"));
      }
      else if (coord_x > Ibox[0] && coord_x < Ibox[1] && coord_y > Ibox[2] && coord_y < Ibox[3]) {
        select = 3;
        tft.cursorToXY(140, 200); tft.setTextScale(1);
        tft.print(F("3"));
      }
      else if (coord_x > Dbox[0] && coord_x < Dbox[1] && coord_y > Dbox[2] && coord_y < Dbox[3]) {
        select = 4;
        tft.cursorToXY(140, 200); tft.setTextScale(1);
        tft.print(F("4"));
      }

      if (coord_x > increasebox[0] && coord_x < increasebox[1] && coord_y > increasebox[2] && coord_y < increasebox[3]) {
        if (select == 1)
        {
          Setpoint = Setpoint + 0.01;
        }
        else if (select == 2)
        {
          Kp = Kp + 0.01;
        }
        else if (select == 3) {
          Ki = Ki + 0.01;
        }
        else if (select == 4)
        {
          Kd = Kd + 0.01;
        }
      }    else if (coord_x > decreasebox[0] && coord_x < decreasebox[1] && coord_y > decreasebox[2] && coord_y < decreasebox[3]) {
        if (select == 1)
        {
          Setpoint = Setpoint - 0.01;
        }
        else if (select == 2)
        {
          Kp = Kp - 0.01;
          if (Kp < 0) {
            Kp = 0;
          }
        }
        else if (select == 3) {
          Ki = Ki - 0.01;
          if (Ki < 0) {
            Ki = 0;
          }
        }
        else if (select == 4)
        {
          Kd = Kd - 0.01;
          if (Kd < 0) {
            Kd = 0;
          }
        }
      }
    }
  }
  myPID0.SetTunings(Kp, Ki, Kd);
}




void printmessage(char message)
{
  tft.cursorToXY(140, 200); tft.setTextScale(1);
  tft.print(F(message));
}

void screenupdate(void)
{
  if (redraw == 0)
  {
  }
  else if (redraw == 1)
  {
    redraw = 0;
    if (screen == 0)
    {
      tft.fillScreen(ILI9341_BLACK);
      //      tft.drawImage(dollar, 180, 130, dollarWidth, dollarHeight);
      labeldrawer();
    } else if (screen == 1)
    {
      tft.fillScreen(ILI9341_BLACK);
      labeldrawer2();
    }
  }
}


void printStatus()
{
  Serial.println("NAME     Vref  Gain  PowerDown  Value");
  for (int channel = 0; channel <= 3; channel++)
  {
    Serial.print("DAC");
    Serial.print(channel, DEC);
    Serial.print("   ");
    Serial.print("    ");
    Serial.print(dac.getVref(channel), BIN);
    Serial.print("     ");
    Serial.print(dac.getGain(channel), BIN);
    Serial.print("       ");
    Serial.print(dac.getPowerDown(channel), BIN);
    Serial.print("       ");
    Serial.println(dac.getValue(channel), DEC);

    Serial.print("EEPROM");
    Serial.print(channel, DEC);
    Serial.print("    ");
    Serial.print(dac.getVrefEp(channel), BIN);
    Serial.print("     ");
    Serial.print(dac.getGainEp(channel), BIN);
    Serial.print("       ");
    Serial.print(dac.getPowerDownEp(channel), BIN);
    Serial.print("       ");
    Serial.println(dac.getValueEp(channel), DEC);
  }
  Serial.println(" ");
}

void printStatus2()
{
  Serial.println("NAME     Vref  Gain  PowerDown  Value");
  for (int channel = 0; channel <= 3; channel++)
  {
    Serial.print("DAC");
    Serial.print(channel, DEC);
    Serial.print("   ");
    Serial.print("    ");
    Serial.print(dac2.getVref(channel), BIN);
    Serial.print("     ");
    Serial.print(dac2.getGain(channel), BIN);
    Serial.print("       ");
    Serial.print(dac2.getPowerDown(channel), BIN);
    Serial.print("       ");
    Serial.println(dac2.getValue(channel), DEC);

    Serial.print("EEPROM");
    Serial.print(channel, DEC);
    Serial.print("    ");
    Serial.print(dac2.getVrefEp(channel), BIN);
    Serial.print("     ");
    Serial.print(dac2.getGainEp(channel), BIN);
    Serial.print("       ");
    Serial.print(dac2.getPowerDownEp(channel), BIN);
    Serial.print("       ");
    Serial.println(dac2.getValueEp(channel), DEC);
  }
  Serial.println(" ");
}
