
/********************************************************
   Temperature controller
   To be worked with voltage controlled TEC driver(ADN8843) and
   Adafruit 2.8" touchscreen or ILI9341 library compatible screen.
   Use Programming Port
 ********************************************************/
//General Libraries
#include <stdio.h> // for function sprintf
#include "AH_MCP320x.h" // ADC Library
#include "dollar.h" // Saffman lab logo.
// Display Libraries
#include <SPI.h>
#include <ILI9341_due_config.h>
#include <ILI9341_due.h>
#include "fonts\Arial14.h"
//#include "dollar.h" // Saffman lab logo.
#include <Wire.h>      // this is needed for FT6206
#include <Adafruit_FT6206.h> // Touch screeb library

#define ADC_CS 53 // Chip Select pin for ADC 1
#define ADC2_CS 2 // Chip Select pin for ADC 2
#define LDAC2 6 // Chip Select pin for DAC 2
#define TFT_RST 8
#define TFT_DC 9
#define TFT_CS 10 // Chip Select pin for Touchscreen

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

#define HOME 0
#define TEC_A_MAIN 1
#define TEC_B_MAIN 2
#define TEC_A_CONFIG 3
#define TEC_B_CONFIG 4
#define MONITOR 5
#define SYSTEM_CONFIG 6
#define TEC_HIGH_VLIMIT 1.5
#define TEC_LOW_VLIMIT -1.5

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
double Setpoint_A = 18, Setpoint_B = 20;
double Input_A, Output_A, Input_B, Output_B;
double thermvoltage_A[6], thermvoltage_B[6];
double thermresistance_A, thermresistance_A2,thermresistance_A3, thermresistance_B, thermresistance_B2,thermresistance_B3;
double temperature_A, temperature_A2,temperature_A3, temperature_B, temperature_B2,temperature_B3;
float Vsense_A, Isense_A, VTEC_A, ITEC_A, Vsense_B, Isense_B, VTEC_B, ITEC_B;
int current_screen = HOME, next_screen = HOME;
int select = 0;
RunningAverage data_A_1(300), data_A_2(300), data_A_1_sink(30), data_A_2_sink(30),data_A_VTEC(20), data_A_ITEC(20),data_A_TH_INT(30); // Put number of samples in the parenthesis
RunningAverage data_B_1(300), data_B_2(300), data_B_1_sink(30), data_B_2_sink(30), data_B_VTEC(20), data_B_ITEC(20),data_B_TH_INT(30);; // Put number of samples in the parenthesis
int samples = 0;
int screen = 0, redraw = 0;
//Specify the links and initial tuning parameters
double Kp_A = 2.5, Kp_B = 2.5;
double Ki_A = 0.1, Ki_B = 0.1;
double Kd_A = 0, Kd_B = 0;
PID myPID_A(&Input_A, &Output_A, &Setpoint_A, Kp_A, Ki_A, Kd_A, DIRECT);
PID myPID_B(&Input_B, &Output_B, &Setpoint_B, Kp_B, Ki_B, Kd_B, DIRECT);
unsigned long clocktime = millis();
unsigned long startTime = millis();
unsigned long screen_startTime=millis();

void setup()
{
  Serial.begin(115200);
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

  pinMode(LDAC2, Output_A);
  digitalWrite(LDAC2, LOW);
  printStatus(); // Print all internal value and setting of Input_A register and EEPROM.
  printStatus2();

  //Running average bin initialization
  data_A_1.clear();
  data_A_2.clear();
  data_A_1_sink.clear();
  data_A_2_sink.clear();
  data_A_VTEC.clear();
  data_A_ITEC.clear();

  data_B_1.clear();
  data_B_2.clear();
  data_B_1_sink.clear();
  data_B_2_sink.clear();
  data_B_VTEC.clear();
  data_B_ITEC.clear();

  //initialize the variables we're linked to

  myPID_A.SetOutputLimits(TEC_LOW_VLIMIT, TEC_HIGH_VLIMIT);
  myPID_A.SetSampleTime(50);
  myPID_A.SetMode(AUTOMATIC);

  myPID_B.SetOutputLimits(TEC_LOW_VLIMIT, TEC_HIGH_VLIMIT);
  myPID_B.SetSampleTime(50);
  myPID_B.SetMode(AUTOMATIC);

  draw_labels(current_screen);

}

void loop()
{
  int count = 0;
  int dactemp, ADCvalue;
  AH_MCP320x ADC_SPI(ADC_CS);
  AH_MCP320x ADC2_SPI(ADC2_CS);

  startTime = millis();
  
  // Main Thermistor sensing
  while (millis() - startTime < 40)
  {
    ADCvalue = ADC_SPI.readCH(ADC_PIN_TH1REF);
    data_A_1.addValue(ADCvalue);
    ADCvalue = ADC_SPI.readCH(ADC_PIN_TH1POS);
    data_A_2.addValue(ADCvalue);

    ADCvalue = ADC2_SPI.readCH(ADC_PIN_TH1REF);
    data_B_1.addValue(ADCvalue);
    ADCvalue = ADC2_SPI.readCH(ADC_PIN_TH1POS);
    data_B_2.addValue(ADCvalue);
    count++;
  }

  // TEC voltage and current sensing
  while (millis() - startTime < 42)
  {
    ADCvalue = ADC_SPI.readCH(ADC_PIN_VTEC);
    data_A_VTEC.addValue(ADCvalue);
    ADCvalue = ADC_SPI.readCH(ADC_PIN_ITEC);
    data_A_ITEC.addValue(ADCvalue);

    ADCvalue = ADC2_SPI.readCH(ADC_PIN_VTEC);
    data_B_VTEC.addValue(ADCvalue);
    ADCvalue = ADC2_SPI.readCH(ADC_PIN_ITEC);
    data_B_ITEC.addValue(ADCvalue);
  }

  // Sink thermistor sensing
  while (millis() - startTime < 44)
  {
    ADCvalue = ADC_SPI.readCH(ADC_PIN_TH2REF);
    data_A_1_sink.addValue(ADCvalue);
    ADCvalue = ADC2_SPI.readCH(ADC_PIN_TH2REF);
    data_B_1_sink.addValue(ADCvalue);

    ADCvalue = ADC_SPI.readCH(ADC_PIN_TH2POS);
    data_A_2_sink.addValue(ADCvalue);
    ADCvalue = ADC2_SPI.readCH(ADC_PIN_TH2POS);
    data_B_2_sink.addValue(ADCvalue);
  }
  // On-board thermistor sensing
    while (millis() - startTime < 46)
  {
    ADCvalue = ADC_SPI.readCH(ADC_PIN_TH_INT);
    data_A_TH_INT.addValue(ADCvalue);
    ADCvalue = ADC2_SPI.readCH(ADC_PIN_TH_INT);
    data_B_TH_INT.addValue(ADCvalue);
  }

  // Data acqusition complete. Now processing the data.
  // First TEC setting
  Vsense_A = data_A_VTEC.getAverage();
  Isense_A = data_A_ITEC.getAverage();
  VTEC_A = 4.0 * ((3.3 * Vsense_A / 4096.0) - 1.25);
  ITEC_A = ((3.3 * Isense_A / 4096.0) - 1.25) / 0.525;

  thermvoltage_A[0] = data_A_1.getAverage();
  thermvoltage_A[1] = data_A_2.getAverage();
  thermvoltage_A[2] = data_A_1_sink.getAverage();
  thermvoltage_A[3] = data_A_2_sink.getAverage();
  thermvoltage_A[4] = 4094;
  thermvoltage_A[5] = data_A_TH_INT.getAverage();
  double VRref_A = thermvoltage_A[0] - thermvoltage_A[1]; // Voltage divided to the 1streference resistor
  double VRref_A2 = thermvoltage_A[2] - thermvoltage_A[3]; // Voltage divided to the 2nd reference resistor
  double VRref_A3 = thermvoltage_A[4] - thermvoltage_A[5]; // Voltage divided to the 2nd reference resistor

  // Get thermistor resistance from the voltage
  thermresistance_A = 10000 * thermvoltage_A[1] / VRref_A;
  thermresistance_A2 = 10000 * thermvoltage_A[3] / VRref_A2;
  thermresistance_A3 = 10000 * thermvoltage_A[5] / VRref_A3;

  temperature_A = thermistor10k(thermresistance_A);
  temperature_A2 = thermistor10k(thermresistance_A2);
  temperature_A3 = thermistor10k(thermresistance_A3);

  Input_A = temperature_A;

  // Secondary TEC driver setting
  Vsense_B = data_B_VTEC.getAverage();
  Isense_B = data_B_ITEC.getAverage();
  VTEC_B = 4.0 * ((3.3 * Vsense_B / 4096.0) - 1.25);
  ITEC_B = ((3.3 * Isense_B / 4096.0) - 1.25) / 0.525;

  thermvoltage_B[0] = data_B_1.getAverage();
  thermvoltage_B[1] = data_B_2.getAverage();
  thermvoltage_B[2] = data_B_1_sink.getAverage();
  thermvoltage_B[3] = data_B_2_sink.getAverage();
  thermvoltage_B[4] = 4094;
  thermvoltage_B[5] = data_B_TH_INT.getAverage();
  double VRref_B = thermvoltage_B[0] - thermvoltage_B[1]; // Voltage divided to the 1streference resistor
  double VRref_B2 = thermvoltage_B[2] - thermvoltage_B[3]; // Voltage divided to the 2nd reference resistor
  double VRref_B3 = thermvoltage_B[4] - thermvoltage_B[5]; // Voltage divided to the 2nd reference resistor

  // Get thermistor resistance from the voltage
  thermresistance_B = 10000 * thermvoltage_B[1] / VRref_B;
  thermresistance_B2 = 10000 * thermvoltage_B[3] / VRref_B2;
  thermresistance_B3 = 10000 * thermvoltage_B[5] / VRref_B3;

  temperature_B = thermistor10k(thermresistance_B);
  temperature_B2 = thermistor10k(thermresistance_B2);
  temperature_B3 = thermistor10k(thermresistance_B3);

  Input_B = temperature_B;

  int ENABLE_A = 1;
  int ENABLE_B = 1;
  // COMPUTE PID
  if (ENABLE_A == 1) {
    myPID_A.Compute();
  }

  if (ENABLE_B == 1) {
    myPID_B.Compute();
  }

  int dacvoltage_A = 1250 - 200 * Output_A; // Control voltage go ADN8843
  int dacvoltage_B = 1250 - 200 * Output_B; // Control voltage to go to ADN8843


  dac.voutWrite(DAC_PIN_TMON, dacvoltage_A);
  dac.voutWrite(DAC_PIN_VCON, dacvoltage_A);
  dac2.voutWrite(DAC_PIN_TMON, dacvoltage_B);
  dac2.voutWrite(DAC_PIN_VCON, dacvoltage_B);

  if (next_screen != current_screen) {
    draw_labels(next_screen);
    current_screen = next_screen;
  }
  
  if(millis()-screen_startTime>0){
  drawvalues(current_screen);
  screen_startTime=millis();
  }


  //  touchaction(ctp.touched());
  touchaction();
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

double thermistor10k_int(double R)
{
  float A = 0.98750e-3;
  float B = 2.56900e-4;
  float C = 0.00023e-7;
  double logR = log(R);
  double temperature = (1.0 / (A + B * logR + C * logR * logR * logR) ) - 273.15-3.00;
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

void draw_labels(int menu_number) {
  tft.fillScreen(ILI9341_BLACK);
  
   if (menu_number != HOME) {
    tft.drawImage(dollar, 200, 150, dollarWidth, dollarHeight);
  }
  
  if (menu_number == HOME) {
    tft.cursorToXY(40, 25); tft.setTextScale(2);
    tft.print(F("TEC #1"));
    tft.cursorToXY(20, 25 + 55);
    tft.print(F("PID CFG #1"));
    tft.cursorToXY(40, 25 + 55 + 55);
    tft.print(F("MNTR"));
    tft.cursorToXY(195, 25);
    tft.print(F("TEC #2"));
    tft.cursorToXY(180, 25 + 55);
    tft.print(F("PID CFG #2"));
    tft.cursorToXY(190, 25 + 55 + 55);
    tft.print(F("SYSTEM"));

    tft.drawRect(2, 16, 158, 49, ILI9341_WHITE);
    tft.drawRect(2, 16 + 55, 158, 49, ILI9341_WHITE);
    tft.drawRect(2, 16 + 55 + 55, 158, 49, ILI9341_WHITE);
    tft.drawRect(2 + 165, 16, 158, 49, ILI9341_WHITE);
    tft.drawRect(2 + 165, 16 + 55, 158, 49, ILI9341_WHITE);
    tft.drawRect(2 + 165, 16 + 55 + 55, 158, 49, ILI9341_WHITE);

  }else  if (menu_number == TEC_A_MAIN||menu_number == TEC_B_MAIN) {
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

    tft.drawRect(2, 16, 158, 49, ILI9341_WHITE);
    tft.drawRect(2, 16 + 55, 158, 49, ILI9341_WHITE);
    tft.drawRect(2, 16 + 55 + 55, 158, 49, ILI9341_WHITE);
    tft.drawRect(2 + 165, 16, 158, 49, ILI9341_WHITE);
    tft.drawRect(2 + 165, 16 + 55, 158, 49, ILI9341_WHITE);
    tft.drawRect(2 + 165, 16 + 55 + 55, 158, 49, ILI9341_WHITE);

  } else  if (menu_number == TEC_A_CONFIG||menu_number == TEC_B_CONFIG) {
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
  } else if (menu_number==MONITOR){
    tft.cursorToXY(100, 2); tft.setTextScale(1);
    tft.print(F("Ch.A"));
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
    tft.cursorToXY(150, 45 + 55+55); tft.setTextScale(1);
    tft.print(F(" C "));

    int xshift=160;
    tft.cursorToXY(100+xshift, 2); tft.setTextScale(1);
    tft.print(F("Ch.B"));
    tft.cursorToXY(5+xshift, 25); tft.setTextScale(3);
    tft.print(F("T"));
    tft.cursorToXY(25+xshift, 45); tft.setTextScale(1);
    tft.print(F("set"));
    tft.cursorToXY(50+xshift, 25); tft.setTextScale(3);
    tft.print(F(":"));
    tft.cursorToXY(150+xshift, 45); tft.setTextScale(1);
    tft.print(F(" C "));
    tft.cursorToXY(5+xshift, 25 + 55); tft.setTextScale(3);
    tft.print(F("T"));
    tft.cursorToXY(25+xshift, 45 + 55); tft.setTextScale(1);
    tft.print(F("sen"));
    tft.cursorToXY(50+xshift, 25 + 55); tft.setTextScale(3);
    tft.print(F(":"));
    tft.cursorToXY(150+xshift, 45 + 55); tft.setTextScale(1);
    tft.print(F(" C "));
    tft.cursorToXY(5+xshift, 25 + 55 + 55); tft.setTextScale(3);
    tft.print(F("T"));
    tft.cursorToXY(25+xshift, 45 + 55 + 55); tft.setTextScale(1);
    tft.print(F("sink"));
    tft.cursorToXY(150+xshift, 45 + 55+55); tft.setTextScale(1);
    tft.print(F(" C "));
  } else if (menu_number==SYSTEM_CONFIG){
    tft.cursorToXY(5, 25); tft.setTextScale(3);
    tft.print(F("T"));
    tft.cursorToXY(20, 45); tft.setTextScale(1);
    tft.print(F("PCB#1"));
    tft.cursorToXY(60, 25); tft.setTextScale(3);
    tft.print(F(":"));
    tft.cursorToXY(150, 45); tft.setTextScale(1);
    tft.print(F(" C "));

    int xshift=160;
    tft.cursorToXY(5+xshift, 25); tft.setTextScale(3);
    tft.print(F("T"));
    tft.cursorToXY(20+xshift, 45); tft.setTextScale(1);
    tft.print(F("PCB#2"));
    tft.cursorToXY(60+xshift, 25); tft.setTextScale(3);
    tft.print(F(":"));
    tft.cursorToXY(150+xshift, 45); tft.setTextScale(1);
    tft.print(F(" C "));
  }
  
 
  tft.cursorToXY(1, 225); tft.setTextScale(1);
  tft.print(F("2016-06-25 Build"));
}


void touchaction(void)
{
  if (ctp.touched())  {
    TS_Point p = ctp.getPoint();
    int coord_x = 320 - p.y;
    int coord_y = p.x;
    Serial.print("X:");
    Serial.print(coord_x);
    Serial.print("  Y:");
    Serial.println(coord_y);

    int MAIN_HOME[4] = {200, 150, 120, 90};
    int MAIN_BOX1[4] = {2, 16, 158, 49};
    int MAIN_BOX3[4] = {2, 16 + 55, 158, 49};
    int MAIN_BOX5[4] = {2, 16 + 55 + 55, 158, 49};
    int MAIN_BOX2[4] = {2 + 165, 15, 158, 49};
    int MAIN_BOX4[4] = {2 + 165, 15 + 55, 158, 49};
    int MAIN_BOX6[4] = {2 + 165, 15 + 55 + 55, 158, 49};

    if (box_check(coord_x, coord_y, MAIN_HOME)) {
      next_screen = HOME;
    } else if (box_check(coord_x, coord_y, MAIN_BOX1)) {
      next_screen = TEC_A_MAIN;
    } else if (box_check(coord_x, coord_y, MAIN_BOX2)) {
      next_screen = TEC_B_MAIN;
    } else if (box_check(coord_x, coord_y, MAIN_BOX3)) {
      next_screen = TEC_A_CONFIG;
    } else if (box_check(coord_x, coord_y, MAIN_BOX4)) {
      next_screen = TEC_B_CONFIG;
    } else if (box_check(coord_x, coord_y, MAIN_BOX5)) {
      next_screen = MONITOR;
    } else if (box_check(coord_x, coord_y, MAIN_BOX6)) {
      next_screen = SYSTEM_CONFIG;
    }

    Serial.println(box_check(coord_x, coord_y, HOME));

    tft.drawRect(2, 16, 158, 49, ILI9341_WHITE);
    tft.drawRect(2, 16 + 55, 158, 49, ILI9341_WHITE);
    tft.drawRect(2, 16 + 55 + 55, 158, 49, ILI9341_WHITE);
    tft.drawRect(2 + 165, 16, 158, 49, ILI9341_WHITE);
    tft.drawRect(2 + 165, 16 + 55, 158, 49, ILI9341_WHITE);
    tft.drawRect(2 + 165, 16 + 55 + 55, 158, 49, ILI9341_WHITE);


    myPID_A.SetTunings(Kp_A, Ki_A, Kd_A);
    myPID_B.SetTunings(Kp_B, Ki_B, Kd_B);
  }
}

void drawvalues(int menu_number) {
  if (menu_number == TEC_A_MAIN) {
    draw7Number((int)Setpoint_A, 60, 20, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(round(1000 * Setpoint_A) % 1000, 110, 35, 1, ILI9341_WHITE, ILI9341_BLACK, 3);
    draw7Number(floor(temperature_A), 60, 20 + 55, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)1000 * temperature_A - 1000 * (int)temperature_A), 110, 35 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 3);
    draw7Number(floor(temperature_A2), 60, 20 + 55 + 55, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)1000 * temperature_A2 - 1000 * (int)temperature_A2), 110, 35 + 55 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 3);
    draw7Number((int)(VTEC_A * 100.0), 60 + 150, 20, 1, ILI9341_WHITE, ILI9341_BLACK, -4);
    draw7Number((int)(ITEC_A * 100.0), 60 + 150, 20 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, -4);
  }
  else if (menu_number == TEC_B_MAIN) {
    draw7Number((int)Setpoint_B, 60, 20, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(round(1000 * Setpoint_B) % 1000, 110, 35, 1, ILI9341_WHITE, ILI9341_BLACK, 3);
    draw7Number(floor(temperature_B), 60, 20 + 55, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)1000 * temperature_B - 1000 * (int)temperature_B), 110, 35 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 3);
    draw7Number(floor(temperature_B2), 60, 20 + 55 + 55, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)1000 * temperature_B2 - 1000 * (int)temperature_B2), 110, 35 + 55 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 3);
    draw7Number((int)(VTEC_B * 100.0), 60 + 150, 20, 1, ILI9341_WHITE, ILI9341_BLACK, -4);
    draw7Number((int)(ITEC_B * 100.0), 60 + 150, 20 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, -4);
  }
  else if (menu_number == TEC_A_CONFIG)
  {
    draw7Number((int)Setpoint_A, 60, 20, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(round(1000 * Setpoint_A) % 1000, 110, 35, 1, ILI9341_WHITE, ILI9341_BLACK, 3);
    draw7Number(floor(Kp_A), 60, 20 + 55, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)100 * Kp_A - 100 * (int)Kp_A), 110, 35 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 2);
    draw7Number((int)Ki_A, 60 + 150, 20, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)100 * Ki_A - 100 * (int)Ki_A), 110 + 150, 35, 1, ILI9341_WHITE, ILI9341_BLACK, 2);
    draw7Number((int)Kd_A, 60 + 150, 20 + 55, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)100 * Kd_A - 100 * (int)Kd_A), 110 + 150, 35 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 2);
  }
  else if (menu_number == TEC_B_CONFIG)
  {
    draw7Number((int)Setpoint_B, 60, 20, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(round(1000 * Setpoint_B) % 1000, 110, 35, 1, ILI9341_WHITE, ILI9341_BLACK, 3);
    draw7Number(floor(Kp_B), 60, 20 + 55, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)100 * Kp_B - 100 * (int)Kp_B), 110, 35 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 2);
    draw7Number((int)Ki_B, 60 + 150, 20, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)100 * Ki_B - 100 * (int)Ki_B), 110 + 150, 35, 1, ILI9341_WHITE, ILI9341_BLACK, 2);
    draw7Number((int)Kd_B, 60 + 150, 20 + 55, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)100 * Kd_B - 100 * (int)Kd_B), 110 + 150, 35 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 2);
  }
    else if (menu_number == MONITOR)
  {
    int xshift=160;
    draw7Number((int)Setpoint_A, 60, 20, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(round(1000 * Setpoint_A) % 1000, 110, 35, 1, ILI9341_WHITE, ILI9341_BLACK, 3);
    draw7Number(floor(temperature_A), 60, 20 + 55, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)1000 * temperature_A - 1000 * (int)temperature_A), 110, 35 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 3);
    draw7Number(floor(temperature_A2), 60, 20 + 55 + 55, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)1000 * temperature_A2 - 1000 * (int)temperature_A2), 110, 35 + 55 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 3);
    
    draw7Number((int)Setpoint_B, 60+xshift, 20, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(round(1000 * Setpoint_B) % 1000, 110+xshift, 35, 1, ILI9341_WHITE, ILI9341_BLACK, 3);
    draw7Number(floor(temperature_B), 60+xshift, 20 + 55, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)1000 * temperature_B - 1000 * (int)temperature_B), 110+xshift, 35 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 3);
    draw7Number(floor(temperature_B2), 60+xshift, 20 + 55 + 55, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(abs((int)1000 * temperature_B2 - 1000 * (int)temperature_B2), 110+xshift, 35 + 55 + 55, 1, ILI9341_WHITE, ILI9341_BLACK, 3);
  }
      else if (menu_number == SYSTEM_CONFIG)
  {
    draw7Number((int)temperature_A3, 70, 20, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(round(100 * temperature_A3) % 100, 120, 35, 1, ILI9341_WHITE, ILI9341_BLACK, 2);
    int xshift=160;
    draw7Number((int)temperature_B3, 70+xshift, 20, 2, ILI9341_WHITE, ILI9341_BLACK, -2);
    draw7Number(round(100 * temperature_B3) % 100, 120+xshift, 35, 1, ILI9341_WHITE, ILI9341_BLACK, 2);
    

  }
  tft.cursorToXY(270, 220); tft.setTextScale(1);
  tft.print(millis() - startTime);
  tft.print(F("ms"));
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
      draw_labels(TEC_A_MAIN);
    } else if (screen == 1)
    {
      tft.fillScreen(ILI9341_BLACK);
      draw_labels(TEC_A_CONFIG);
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

int box_check(int coord_x, int coord_y, int box[4]) {
  if (coord_x > box[0] && coord_x < box[0] + box[2] && coord_y > box[1] && coord_y < box[1] + box[3]) {
    return 1;
  } else {
    return 0;
  }
}

