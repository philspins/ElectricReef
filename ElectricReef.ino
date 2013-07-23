//---------------------------------------------------------
// Stilo v.3.0 Aquarium Controller
//
// http://code.google.com/p/stilo/
//---------------------------------------------------------
//
// Libraries used:
// UTFT and UTouch by Henning Karlsen
// http://www.henningkarlsen.com/electronics
//
// LED controlling algorithm is based on Krusduino by Hugh Dangerfield
// http://code.google.com/p/dangerduino/
//
// Moonphases algorithm by Fraser Farrell 
// http://www.delphigroups.info/2/03/20272.html
// Moonrise / moonset algorithm by Stephen R. Schmitt 
// http://mysite.verizon.net/res148h4j/javascript/script_moon_rise_set2.html
//
//-----------------------------------------------------------
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation version 3
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
//------------------------------------------------------------------------------


#include <UTFT.h>
#include <UTouch.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <EEPROM.h>
#include <OneWire.h>
#include "writeAnything.h"
#include "moon.h"

//Temperature libs I'm not using
//#include <DS1307new.h>
//#include <DallasTemperature.h>



//Change screen type and pins if required
UTFT myGLCD(SSD1289,38,39,40,41);   //
//UTFT myGLCD(HX8347A,38,39,40,41);
UTouch      myTouch(46,45,44,43,42);

// Pins for temperature sensors
//#define ONE_WIRE_BUS_W 47         //water sensor
//#define ONE_WIRE_BUS_H 48         //LEDs heatsink sensor

// Setup a oneWire instance to communicate with any OneWire devices
//OneWire oneWireW(ONE_WIRE_BUS_W);
//OneWire oneWireH(ONE_WIRE_BUS_H);

// Pass our oneWire reference to Dallas Temperature. 
//DallasTemperature sensorW(&oneWireW);      //water sensor
//DallasTemperature sensorH(&oneWireH);      //heatsink sensor

#define LARGE true
#define SMALL false

// Declare which fonts are used
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
//extern uint8_t SevenSegNumFont[];

extern unsigned int NewMoon[0x9C4];
extern unsigned int WaxiCresc[0x9C4];
extern unsigned int FirstQuarter[0x9C4];
extern unsigned int WaxiGibb[0x9C4];
extern unsigned int FullMoon[0x9C4];
extern unsigned int WaniGibb[0x9C4];
extern unsigned int ThirdQuarter[0x9C4];
extern unsigned int WaniCresc[0x9C4];


//-----------------------other pins---------------------------------
const byte ledPinCh1 = 6;        // channel 1 LEDs pin
const byte ledPinCh2 = 7;        // channel 2 LEDs pin
const byte ledPinCh3 = 8;        // channel 3 LEDs pin
const byte ledPinCh4 = 11;       // channel 4 LEDs pin
//const byte ledPinCh5 = 12;       // channel 5 LEDs pin
const byte moonlightPin = 13;    // moonlights pin
const byte LCDbrightPin = 10;    //pin to controll LCD brightness
//const byte AlarmBuzzPin = 4;     //pin for alarm buzzer - unused
//const byte OC3B_Pin = 5;         //pin 5 can't be used as its register is used to controll PWM frequency on timer 3
//const byte fan1PWMpin = 2;       //PWM (25kHz) for heatsink fan - unused
//const byte fan2PWMpin = 3;       //PWM (25kHz) - unused
//const byte fan1TranzPin = 49;    //heatsink fan on/off - unused
//const byte ATOswitchPin = 19;    //ATO water level switch - unused
//const byte ATOpumPin = A1;       //ATO water pump - unused
const byte sensorW = A2;  	 //water sensor
const byte sensorWPower = 2;		 	 //water sensor power


//-------------------------Settings---------------------------------
const byte numberOfCh = 4;                                        //Number of LEDs channels to control
const char *namesCh[] = {"  ", "RB", "WH", "UV", "OCW"};     	  //Names of each channel on display
const byte rgbCh1[] = {30, 30, 255};                              //rgb colour of each channel
const byte rgbCh2[] = {255, 255, 255};
const byte rgbCh3[] = {176, 28, 203};
const byte rgbCh4[] = {255, 255, 225};
const boolean BUCKPUCK  = false;          //For MeanWell led drivers change to "false"
const boolean readEEonStart = 1;          //read settings from EEPROM on start
//const int fanHyster = 300;                //difference between fan on and fan off temperature
const float setTempC = 27.0;              // desired water temp for alarm widget
//const float AlarmOffTemp = 0.0;           // +- value for water temp alarm 
//const int HsinkTempMin = 30;              //temp when heatsink fan starts
//const int HsinkTempMax = 40;              //temp when heatsink fan is 100% 
//const int HsinkTempCutOff = 70;           //LEDs temp when to start decreasing LEDs output
//int ATOmax = 5;                           //max time allowed for ato pump
const boolean moonTimesReal = 1;          //use calculated real times for moonrise and moonset
const int setMoonrise = 18;               //time to turm on moonlight
const int setMoonset = 9;                 //time to turm off moonlight
const byte setNewMoonLight = 10;          //New Moon light output value
const byte setFullMoonLight = 80;         //Full Moon light output value
const int latitude = 52;                  //important bit, negative if south 
const int longtitude = 0;                 //not so important bit, negative if west
byte LCDbright = 70;                      //LCD brightness


// Channel 1 leds output values %:
byte ch1led[96] = {
  0, 0, 0, 0, 0, 0, 0, 0,  //0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,  //2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,  //4 - 5
  0, 0, 0, 0, 0, 0, 11, 11,  //6 - 7
  11, 11, 11, 12, 12, 13, 13, 14,  //8 - 9
  16, 16, 16, 16, 20, 25, 30, 35,  //10 - 11
  40, 47, 47, 47, 47, 47, 47, 47,  //12 - 13
  47, 47, 47, 47, 47, 47, 47, 47,  //14 - 15
  47, 47, 47, 47, 47, 47, 47, 47,  //16 - 17
  47, 47, 47, 47, 47, 40, 35, 30,  //18 - 19
  25, 20, 15, 11, 11, 11, 11, 11,  //20 - 21
  0, 0, 0, 0, 0, 0, 0, 0    //22 - 23
};  

// Channel 2 leds output values %:
byte ch2led[96] = {
  0, 0, 0, 0, 0, 0, 0, 0,  //0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,  //2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,  //4 - 5
  0, 0, 0, 0, 0, 0, 0, 0,  //6 - 7
  0, 0, 0, 0, 0, 0, 0, 0,  //8 - 9
  0, 0, 0, 11, 14, 20, 26, 37,  //10 - 11
  47, 55, 55, 55, 55, 55, 55, 55,  //12 - 13
  55, 55, 55, 55, 55, 55, 55, 55,  //14 - 15
  55, 55, 55, 55, 55, 55, 55, 55,  //16 - 17
  55, 55, 55, 55, 47, 37, 26, 19,  //18 - 19
  11, 0, 0, 0, 0, 0, 0, 0,  //20 - 21
  0, 0, 0, 0, 0, 0, 0, 0    //22 - 23
};

// Channel 3 leds output values %:
byte ch3led[96] = {
  0, 0, 0, 0, 0, 0, 0, 0,  //0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,  //2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,  //4 - 5
  0, 0, 0, 0, 0, 0, 11, 11,  //6 - 7
  11, 11, 11, 12, 12, 13, 13, 14,  //8 - 9
  16, 16, 16, 16, 20, 25, 30, 45,  //10 - 11
  60, 80, 100, 100, 100, 100, 100, 100,  //12 - 13
  100, 100, 100, 100, 100, 100, 100, 100,  //14 - 15
  100, 100, 100, 100, 100, 100, 100, 100,  //16 - 17
  100, 100, 100, 100, 70, 50, 35, 30,  //18 - 19
  25, 20, 15, 11, 11, 11, 11, 11,  //20 - 21
  0, 0, 0, 0, 0, 0, 0, 0    //22 - 23
};  

// Channel 4 leds output values %:
byte ch4led[96] = {
  0, 0, 0, 0, 0, 0, 0, 0,  //0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,  //2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,  //4 - 5
  0, 0, 0, 0, 0, 0, 11, 11,  //6 - 7
  11, 11, 11, 12, 12, 13, 13, 14,  //8 - 9
  16, 16, 16, 16, 20, 25, 30, 35,  //10 - 11
  40, 47, 47, 47, 47, 47, 47, 47,  //12 - 13
  47, 47, 47, 47, 47, 47, 47, 47,  //14 - 15
  47, 47, 47, 47, 47, 47, 47, 47,  //16 - 17
  47, 47, 47, 47, 47, 40, 35, 30,  //18 - 19
  25, 20, 15, 11, 11, 11, 11, 11,  //20 - 21
  0, 0, 0, 0, 0, 0, 0, 0    //22 - 23
};  

// Channel 5 leds output values %:
byte ch5led[96] = {
  0, 0, 0, 0, 0, 0, 0, 0,  //0 - 1
  0, 0, 0, 0, 0, 0, 0, 0,  //2 - 3
  0, 0, 0, 0, 0, 0, 0, 0,  //4 - 5
  0, 0, 0, 0, 0, 0, 11, 11,  //6 - 7
  11, 11, 11, 12, 12, 13, 13, 14,  //8 - 9
  16, 16, 16, 16, 20, 25, 27, 27,  //10 - 11
  27, 27, 27, 27, 27, 27, 27, 27,  //12 - 13
  27, 27, 27, 27, 27, 27, 27, 27,  //14 - 15
  27, 27, 27, 27, 27, 27, 27, 27,  //16 - 17
  27, 27, 27, 27, 27, 25, 25, 20,  //18 - 19
  20, 20, 15, 11, 11, 11, 11, 11,  //20 - 21
  0, 0, 0, 0, 0, 0, 0, 0    //22 - 23
};  


//--------------------variables---------------------------------
                    
// 0-main screen, 1-menu, 2-clock setup, 3-LED test, 4-temp control, 5-LED settings,
// 6-LED values table, 7-LED changes, 8-ATO, 9-Moonlight settings, 10-General settings, 11-LED color mixer
byte dispScreen=0;                  
int x, y;                                    //touch coordinates
unsigned long prevMillis05 = 0;
unsigned long prevMillis1 = 0;
unsigned long prevMillis5 = 0;
unsigned long prevMillis60 = 0;
unsigned long prevMillis600 = 0;
//DeviceAddress waterSensor, ledSensor;       // temperature sensors inernal addresses
float tempC = 0;                            // water temperature
//float tempH = 0;                            // heatsink temperature
boolean tempCorHflag=0;                     //refresh water or led temperature
byte tempAlarmFlag=0;                       //1 if alarm on, 2 if alarm was on
//boolean AlarmSoundFlag = 0;                 //1 if alarm sound is on
//int Fan1PWMval = 0;                         //PWM value for fan1
//int Fan2PWMval = 0;                         //PWM value for fan2
//byte LEDcutOff = 100;                       //decrease LEDs output modificator if temp is over tempHcuttOff
//boolean LEDcutFlag = 0;                     //1 if heatsink temp was over HsinkTempCutOff
int LedChangTime = 0;                       //LED change page, time and values
int selectChannel;                          //display LED screen for chosen channel
char tChar30[30];                          
byte LEDtestFlag = 0;                       //for testing leds, 0-off, 1-test running, 2-test stopped
int min_cnt ;
byte tled[96];                              //temporary array to hold changed led values
byte *LEDval_arr[] = {ch1led, ch2led, ch3led, ch4led};                //array to hold led light values
byte LEDch_out[4]; 
byte prevLEDch_out[4]; 
byte LEDpin[] = {ledPinCh1, ledPinCh2, ledPinCh3, ledPinCh4};     
char ch1Descr[15],  ch2Descr[15], ch3Descr[15], ch4Descr[15];   //LED channels descriptions
const byte *rgbChann[] = {rgbCh1, rgbCh2, rgbCh3, rgbCh4};            //LED channels colors
char *Day[] = {"Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};
char *Mon[] = {"","Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
//byte ATOflag = 1;               //0-ATO off, <=1 ATO on: 1-switch off, 2-switch on, 3-fail(switch on for too long)
//long ATOstart = 0;              //millis when ato pump started
//byte tmpATOflag;                //temp values
//int tmpATOmax;
boolean moonUp = 0;             //true if moon is up
char riseSet_descr[15];         //for real times mode
char *MPhase[] = {"","New Moon","Waxing Crescent","First Quarter","Waxing Gibbous","Full Moon","Waning Gibbous","Third Quarter","Waning Crescent"};
unsigned int *MoonPics[] = {NewMoon, WaxiCresc, FirstQuarter, WaxiGibb, FullMoon, WaniGibb, ThirdQuarter, WaniCresc};         
long rise_time, set_time;       //moonrise and moonset times
byte tmpLCDbright; 

struct RTC_T
{  
  int tHour;
  int tMinute;
  int tDow;
  int tDay;
  int tMonth;
  int tYear;
} tmpRTC, prevRTC;

struct tempValues_t
{
  float tempSet;
  float tempOff;
  int tempLEDmin;
  int tempLEDmax;
  int tempLEDcut;
} tempVal, tmpTempVal;    //temperature values

struct tempSett_t
{
  int tempset;
  int tempoff;
//  int tempLEDmin;
//  int tempLEDmax;
//  int tempLEDcut;
} tempSettings;           //for use with EEPROM

/*
struct ato_t
{
  byte tATOflag;
  int tATOmax;
} atoSettings;
*/

struct moonSet_t
{
  boolean timesReal;
  byte moonrise;
  byte moonset;
  byte newMoonLight;
  byte fullMoonLight;
} moonSett, tmpMoonSett;

//--------------------buttons------------------------
const int canC[]= {160,194,317,225};       //standard cancel
const int prOK[]= {2,194,158,225};         //standard ok

//-----------------main screen----------------------
const int alaR[]= {7, 195, 173, 219};      //confirm alarm

//------------------menu----------------------------
const int tanD[]= {2, 21, 158, 59};        //time and date
const int temC[]= {2, 61, 158, 99};        //temp. settings
//const int atoS[]= {2, 101, 158, 139};      //ATO settings
const int scrS[]= {2, 141, 158, 179};      //screen settings
const int ledW[]= {160, 21, 317, 59};      //white LEDs values settings
const int tesT[]= {160, 61, 317, 99};      //test LED
const int colM[]= {160, 101, 317, 139};    //Color mixer

//------------------Time and Date------------
const int houP[]= {130, 22};               //hour up
const int houM[]= {130, 71};               //hour down
const int minP[]= {190, 22};               //minute up
const int minM[]= {190, 71};               //minute down

const int dayU[]= {120, 111};              //day up
const int dayD[]= {120, 160};              //day down
const int monU[]= {179, 111};              //month up
const int monD[]= {179, 160};              //month down
const int yeaU[]= {250, 111};              //year up
const int yeaD[]= {250, 160};              //year down

//------------------Temperature Settings-----------------
const int temP[]= {138,28};                //temp. plus
const int temM[]= {138,50};                //temp. minus
const int offP[]= {277,28};                //offset plus
const int offM[]= {277,50};                //offset minus
//const int hemP[]= {118,87};                //Heatsink fans min temperature plus
//const int hemM[]= {118,109};               //heatsink fans min temperature minus
//const int hetP[]= {257,87};                //Heatsink fans max temperature plus
//const int hetM[]= {257,109};               //heatsink fans max temperature minus
//const int hecP[]= {162,146};               //LEDs cut off temperature plus
//const int hecM[]= {162,168};               //LEDs cut off temperature minus

/*-----------------------------------ATO settings------------------
const int atoO[]= {150, 30};               //ATO on/off switch
const int atoP[]= {176,93};                //ATO time plus
const int atoM[]= {176,115};               //ATO time minus
*/

//----------------------------------LEDs menu-------------------
const int ch1L[]= {2, 21, 158, 59};        //channel 1 light output
const int ch3L[]= {2, 61, 158, 99};        //channel 3 light output
//const int ch5L[]= {2, 101, 158, 139};      //channel 5 light output
const int ch2L[]= {160, 21, 317, 59};      //channel 2 light output
const int ch4L[]= {160, 61, 317, 99};      //channel 4 light output
const int mooL[]= {160, 141, 317, 179};    //moonlight settings

//-----------------------------------test LEDs------------------
const int stsT[]= {76, 158, 243, 191};     //start/stop
const int tenM[]= {2, 158, 74, 191};       //-10s
const int tenP[]= {245, 158, 317, 191};    //+10s

//----------------------------------Moonlight Settings---------------
const int morO[]= {170, 25};               //moonrise/set real time on/off
const int morP[]= {115, 74};               //moonrise time plus
const int morM[]= {115, 96};               //moonrise time minus
const int mosP[]= {268 ,74};               //moonset time plus
const int mosM[]= {268, 96};               //moonset time minus
const int nmlP[]= {115, 143};              //newmoon light value plus
const int nmlM[]= {115, 165};              //newmoon light value minus
const int fmlP[]= {268, 143};              //fullmoon light value plus
const int fmlM[]= {268, 165};              //fullmoon light value minus

//-------------------------------General settings------------------------
const int gseB[]= {64, 39};                //brightness bar
const int gseS[]= {160, 60, 318, 90};      //Save
const int gseL[]= {160, 150, 318, 180};    //Load



//////////////////////////////////////////////////////////////////////////////////////////////
//                                    FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////

void initSettings()
{
  moonSett.timesReal = moonTimesReal;
  moonSett.moonrise = setMoonrise;
  moonSett.moonset = setMoonset;
  moonSett.newMoonLight = setNewMoonLight;
  moonSett.fullMoonLight = setFullMoonLight;
  
  strcat(ch1Descr, "CHANNEL 1 (");
  strcat(ch1Descr, namesCh[1]);
  strcat(ch1Descr, ")" );
  strcat(ch2Descr, "CHANNEL 2 (");
  strcat(ch2Descr, namesCh[2]);
  strcat(ch2Descr, ")" );
  strcat(ch3Descr, "CHANNEL 3 (");
  strcat(ch3Descr, namesCh[3]);
  strcat(ch3Descr, ")" );
  strcat(ch4Descr, "CHANNEL 4 (");
  strcat(ch4Descr, namesCh[4]);
  strcat(ch4Descr, ")" );
//  strcat(ch5Descr, "CHANNEL 5 (");
//  strcat(ch5Descr, namesCh[5]);
//  strcat(ch5Descr, ")" );
  
  tempVal.tempSet = setTempC;
  tempVal.tempOff = AlarmOffTemp;
//  tempVal.tempLEDmin = HsinkTempMin;
//  tempVal.tempLEDmax = HsinkTempMax;
//  tempVal.tempLEDcut = HsinkTempCutOff;
}

void clearScreen()
{
 myGLCD.setColor(0, 0, 0);
 myGLCD.fillRect(1, 1, 318, 225); 
}

void printHeader(char* headline)
{
     changeFont(SMALL, 255, 0, 0, 255, 0, 0);
     myGLCD.fillRect (1, 1, 318, 15);
     myGLCD.setColor(255, 255, 255);     
     myGLCD.print(headline, CENTER, 2);   
}

void printButton(char* text, int x1, int y1, int x2, int y2)
{
  int stl = strlen(text);
  int fx, fy;
 
  fx = x1+(((x2 - x1)-(stl*8))/2);
  fy = y1+(((y2 - y1-1)-8)/2);
  
  changeFont(SMALL, 255, 255, 255, 0, 0, 0);
  myGLCD.print(text, fx, fy);
}

void hlightButton(char* text, int x1, int y1, int x2, int y2)
{
  int stl = strlen(text);
  int fx, fy;
  
  fx = x1+(((x2 - x1)-(stl*8))/2);
  fy = y1+(((y2 - y1-1)-8)/2);
  
  myGLCD.setColor(0, 0, 100);
  myGLCD.fillRect (x1+4, y1+4, x2-4, y2-4);
  changeFont(SMALL, 255, 255, 255, 0, 0, 100);
  myGLCD.print(text, fx, fy);
  while (myTouch.dataAvailable()) {
    myTouch.read(); 
  }
  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRect (x1+4, y1+4, x2-4, y2-4);
  changeFont(SMALL, 255, 255, 255, 0, 0, 0);
  myGLCD.print(text, fx, fy);
}

void printUpButton(int x, int y)           //size 29x23px
{ 
  myGLCD.setColor(0, 0, 125);
  myGLCD.drawLine(x, y+22, x+29, y+22);
  myGLCD.drawLine(x, y+23, x+29, y+23);
  myGLCD.setColor(100, 100, 200);
  for (int i=0; i<15; i++)
     myGLCD.drawLine(x+4+(i/1.5), y+17-i, x+25-(i/1.5), y+17-i);
}

void printDownButton(int x, int y)           //size 29x23px
{
  myGLCD.setColor(0, 0, 125);
  myGLCD.drawLine(x, y, x+29, y);
  myGLCD.drawLine(x, y+1, x+29, y+1);
  myGLCD.setColor(100, 100, 200);
  for (int i=0; i<15; i++)
     myGLCD.drawLine(x+4+(i/1.5), y+7+i, x+25-(i/1.5), y+7+i);
}

void hlightUpButton(int x, int y)           //
{
  myGLCD.setColor(0, 0, 125);
  for (int i=0; i<15; i++)
     myGLCD.drawLine(x+4+(i/1.5), y+17-i, x+25-(i/1.5), y+17-i);
  while (myTouch.dataAvailable()) {
    myTouch.read(); 
  }
  myGLCD.setColor(128, 128, 255);
  for (int i=0; i<15; i++)
     myGLCD.drawLine(x+4+(i/1.5), y+17-i, x+25-(i/1.5), y+17-i);
}

void hlightDownButton(int x, int y)           //
{
  myGLCD.setColor(0, 0, 125);
  for (int i=0; i<15; i++)
     myGLCD.drawLine(x+4+(i/1.5), y+7+i, x+25-(i/1.5), y+7+i);
  while (myTouch.dataAvailable()) {
    myTouch.read(); 
  }
  myGLCD.setColor(128, 128, 255);
  for (int i=0; i<15; i++)
     myGLCD.drawLine(x+4+(i/1.5), y+7+i, x+25-(i/1.5), y+7+i);
}

void printOnOffbutton(boolean onOff, int x1, int y1)
{
  if (onOff == 1) {                                 //switch is on
    myGLCD.setColor(0, 0, 0);
    myGLCD.fillRoundRect (x1+1, y1+1, x1+40, y1+24);
    myGLCD.setColor(0, 0, 125);
    myGLCD.fillRoundRect (x1+40, y1, x1+80, y1+25);
    myGLCD.drawLine(x1+3, y1, x1+37, y1);
    myGLCD.drawLine(x1, y1+3, x1, y1+22);
    myGLCD.drawLine(x1+3, y1+25, x1+37, y1+25);    
    changeFont(SMALL, 255, 255, 255, 0, 0, 125);
    myGLCD.print("ON", x1+55, y1+7);
    changeFont(SMALL, 128, 128, 128, 0, 0, 0);
    myGLCD.print("OFF", x1+10, y1+7);
  }
  else if (onOff == 0) {                             //switch is off
    myGLCD.setColor(0, 0, 0);
    myGLCD.fillRoundRect (x1+40, y1, x1+80, y1+25);
    myGLCD.setColor(0, 0, 125);
    myGLCD.fillRoundRect (x1+1, y1+1, x1+40, y1+24);
    myGLCD.drawLine(x1+43, y1, x1+77, y1);
    myGLCD.drawLine(x1+80, y1+3, x1+80, y1+22);
    myGLCD.drawLine(x1+43, y1+25, x1+77, y1+25);
    changeFont(SMALL, 128, 128, 128, 0, 0, 0);
    myGLCD.print("ON", x1+55, y1+7);
    changeFont(SMALL, 255, 255, 255, 0, 0, 125);
    myGLCD.print("OFF", x1+10, y1+7);
  }
}

void printVBar(int val, int x1, int y1, const byte rgb[])
{
  myGLCD.setColor(0, 0, 125);
  myGLCD.drawRect (x1, y1+22, x1+29, y1+124);
  myGLCD.setColor(rgb[0], rgb[1], rgb[2]);
  myGLCD.fillRect (x1+1, y1+(124-val), x1+28, y1+124);
  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRect (x1+1, y1+22, x1+28, y1+(124-val));
  printUpButton(x1, y1);
  printDownButton(x1, y1+124);
  changeFont(SMALL, 255, 255, 255, 0, 0, 0);    
  if (val > -1) 
    myGLCD.printNumI(val, x1+16-(intlen(val)*4), y1+66);
} 

void changeFont(boolean font, byte cr, byte cg, byte cb, byte br, byte bg, byte bb)
{
  myGLCD.setBackColor(br, bg, bb);    //font background 
  myGLCD.setColor(cr, cg, cb);        //font colour 
  if (font==LARGE)
    myGLCD.setFont(BigFont);          //font size 
  else if (font==SMALL)
    myGLCD.setFont(SmallFont);
}

void drawLedBar(int bary, byte ch)
{
  int barLength;
  char barLevel[5];    
  
  itoa(LEDch_out[ch-1], barLevel, 10);
  strcat(barLevel, "%");
  barLength = LEDch_out[ch-1]*2;                               //lenght of bar, 100% = 200px
  
  myGLCD.setColor(0, 0, 0);
  myGLCD.fillRect(41, bary, 318, (bary+16));                   //hide end of last bar
  changeFont(LARGE, rgbChann[ch-1][0], rgbChann[ch-1][1], rgbChann[ch-1][2], 0, 0, 0);
  myGLCD.print(namesCh[ch], 4, bary);
  myGLCD.drawLine(40, (bary+1), 40, (bary+15));
  myGLCD.fillRect(42, (bary+4), (barLength+42), (bary+12));            //draw the bar
  myGLCD.print(barLevel, (barLength+50), (bary));                      // print led value
}

void printTime(int thour, int tminute, int posx, int posy)
{
  char tmpTime[6], charT[3];

  tmpTime[0] = '\0';

  if (thour>=0 && thour<=9) {          //add space
    strcat(tmpTime, " ");
    itoa(thour, charT, 10);
    strcat(tmpTime, charT);
  }
  else 
    itoa(thour, tmpTime, 10);
  
  strcat(tmpTime, ":");  
  
  if (tminute>=0 && tminute<=9) {         //add 0
    strcat(tmpTime, "0");
    itoa(tminute, charT, 10);
    strcat(tmpTime, charT);
  }
  else {
    itoa(tminute, charT, 10);
    strcat(tmpTime, charT);
  }
    
  myGLCD.print(tmpTime, posx, posy);           // Display time  
}

void printDate(int x, int y) 
{
  char  chDate[25], tmpChar[5];
  
  //strcat(chDate, "  ");
  chDate[0] = '\0';
  strcat(chDate, Day[RTC.dow]);
  strcat(chDate, ", ");
  itoa(RTC.day, tmpChar, 10);
  strcat(chDate, tmpChar);
  strcat(chDate, " ");
  strcat(chDate, Mon[RTC.month]);
  strcat(chDate, " ");
  itoa(RTC.year, tmpChar, 10);
  strcat(chDate, tmpChar);
  strcat(chDate, "   ");

  myGLCD.print(chDate, x, y);            //Display date 
}

int intlen(int number)               //number of digits
{
  int length;
  char tmpChar[5];
  
  itoa(number, tmpChar, 10);
  length = strlen(tmpChar);

  return length;
}

int stringCenter (char *myString, boolean font=false)               //returns half lenght of string in pixels
{
  int chLen;
  
  if (font)                                   //large font
    chLen = (strlen(myString) * 8) -1;
  else                                         //small font
    chLen = (strlen(myString) * 4) -1;
  return chLen;
}


/*-------------------ISR for ATO function----------------
void pumpOn()
{
  if ((digitalRead (ATOswitchPin) == LOW) && (ATOflag == 1)) {     //switch on and ato is on
    digitalWrite (ATOpumPin, HIGH);                                //pump on
   //Serial.println(ATOflag); 
  }
  else {                                                           //switch is off
    digitalWrite (ATOpumPin, LOW);
    if (ATOflag == 2)
      ATOflag = 1;
    //Serial.println(ATOflag);  
  }
}
*/

//---------------------------------------------------------------------------------------------
void checkTempC()
{                                           
/*------------------ORIGINAL TEMPERATURE CODE------------------
 sensorW.requestTemperatures();
 tempC = sensorW.getTempC(waterSensor);           //read water temperature
  
  if (tempVal.tempOff>0) {                        //turn on alarm
     if ((tempC>=(tempVal.tempSet+tempVal.tempOff)) || (tempC<=(tempVal.tempSet-tempVal.tempOff)))
       {
       if (tempAlarmFlag != 1) {
         tempAlarmFlag=1;
         AlarmSoundFlag = true; }
       if (AlarmSoundFlag == true) {
         //digitalWrite(AlarmBuzzPin, HIGH);
         analogWrite(AlarmBuzzPin, 127);
         delay(500);
         analogWrite(AlarmBuzzPin, 0);
         }
       }
  }
     
  if (tempAlarmFlag==1) {
    if (tempC<(tempVal.tempSet+tempVal.tempOff) && tempC>(tempVal.tempSet-tempVal.tempOff)) {
      tempAlarmFlag = 2;
      AlarmSoundFlag = false;
      digitalWrite(AlarmBuzzPin, LOW);                                   //turn off alarm
    }
   if (tempVal.tempOff<0.1) {
     tempAlarmFlag = 0;
     AlarmSoundFlag = false;
     digitalWrite(AlarmBuzzPin, LOW);
   }
  }    
    
  //Serial.println(fanSpeed);
*/

	float v_out;  				 		//voltage output from temp sensor  
  	float temp;							//the final temperature is stored here 
   	v_out = analogRead(sensorW);  			//read the input pin
   	v_out*=.0048; 						//convert ADC points to volts (we are using .0048 because this device is running at 5 volts)
   	v_out*=1000;              			//convert volts to millivolts
   	temp= 0.0512 * v_out -20.5128;		//the equation from millivolts to temperature   
	tempC=temp;						//send back the temp
	
	if (tempVal.tempOff>0) {                        //turn on alarm
      if ((tempC>=(tempVal.tempSet+tempVal.tempOff)) || (tempC<=(tempVal.tempSet-tempVal.tempOff))) {
         if (tempAlarmFlag != 1) {
           tempAlarmFlag=1; }
      }
  	}
	
	if (tempAlarmFlag==1) {
    if (tempC<(tempVal.tempSet+tempVal.tempOff) && tempC>(tempVal.tempSet-tempVal.tempOff)) {
      tempAlarmFlag = 2;
    }
   if (tempVal.tempOff<0.1) {
     tempAlarmFlag = 0;
   }
  } 
} 










//


 


}

/*
void checkTempH()
{
  sensorH.requestTemperatures();
  tempH = (sensorH.getTempC(ledSensor));                            //read heatsink temperature
   
  if ((tempVal.tempLEDcut == 0) || (tempVal.tempLEDcut > tempH)) {           //decreasing LEDs output
    LEDcutOff = 100;
    }
  else if ((tempVal.tempLEDcut > 0) && (tempH > tempVal.tempLEDcut)) { 
    LEDcutOff = 100 - ((tempH - tempVal.tempLEDcut) *10);                    //decrease 10% for each 1degC over limit
    LEDcutFlag = 1;
  }
   
  int fanSpeed = map(int(tempH*10), (tempVal.tempLEDmin*10), (tempVal.tempLEDmax*10), 0, 639);       //---------heatsink fan control
  if (fanSpeed<=0) 
     fanSpeed = 0;
  if (fanSpeed>639)
     fanSpeed=639;
  Fan1PWMval = fanSpeed;
  Fan2PWMval = fanSpeed;
  if (fanSpeed>=fanHyster)                                             //hysteresis
     digitalWrite(fan1TranzPin, HIGH);
  if (fanSpeed<=2)
     digitalWrite(fan1TranzPin, LOW);
}
*/


//--------------------------------EEPROM---------------------------
// addresses:
// 0 - value 121 if channel 1 leds saved
// 1-96 - channel 1 leds output
// 97 - value 122 if channel 2 leds saved
// 98-193 - channel 2 leds output
// 194 - value 123 if channel 3 leds saved
// 195 - 290 - channel 3 leds output
// 291 - value 124 if channel 4 leds saved
// 292 - 387 - channel 4 leds output
// 388 - value 125 if channel 5 leds saved
// 389 - 484 - channel 5 leds output
// 485 - value 126 if temperature settings saved
// 486-506 - temperature settings
// 507 - value 127 if ATO settings saved
// 508-516 - ATO settings
// 517 - value 128 if Moonlight settings saved
// 518-538 - Moonlight settings
// 539 - value 129 if general settings saved
// 540 - General settings (LCD brightness)

void SaveToEEPROM(int what)
{
  switch (what) {
  case 1:                              //save channel 1 LEDs outputs
    EEPROM.write(0, 121);                //to determine if data available in EEPROM
    delay(5);
    for (int i=0; i<96; i++)  {
      EEPROM.write(i+1, ch1led[i]);
      delay(5);
    }
  break;
  case 2:                              //save channel 2 LEDs outputs
    EEPROM.write(97, 122);                 //to determine if data available in EEPROM
    delay(5);
    for (int i=0; i<96; i++)  {
      EEPROM.write(i+98, ch2led[i]);
      delay(5);
    }
  break;
  case 3:                              //save channel 3 LEDs outputs
    EEPROM.write(194, 123);                //to determine if data available in EEPROM
    delay(5);
    for (int i=0; i<96; i++)  {
      EEPROM.write(i+195, ch3led[i]);
      delay(5);
    }
  break;
  case 4:                              //save channel 4 LEDs outputs
    EEPROM.write(291, 124);                //to determine if data available in EEPROM
    delay(5);
    for (int i=0; i<96; i++)  {
      EEPROM.write(i+292, ch4led[i]);
      delay(5);
    }
  break;
/*  case 5:                            //save channel 5 LEDs outputs
    EEPROM.write(388, 125);                //to determine if data available in EEPROM
    delay(5);
    for (int i=0; i<96; i++)  {
      EEPROM.write(i+389, ch5led[i]);
      delay(5);
    }
  break; */
  case 6:                                 //save temperature settings
    EEPROM.write(485, 126);
    tempSettings.tempset = int(tempVal.tempSet*10);
/*    tempSettings.tempoff = int(tempVal.tempOff*10);
    tempSettings.tempLEDmin = tempVal.tempLEDmin;
    tempSettings.tempLEDmax = tempVal.tempLEDmax;
    tempSettings.tempLEDcut = tempVal.tempLEDcut; */
    EEPROM_writeAnything(486, tempSettings);
  break;
/*  case 7:                                   //save ATO settings
    EEPROM.write(507, 127);
    if ((ATOflag == 1) || (ATOflag == 2))
      atoSettings.tATOflag = 1;
    else
      atoSettings.tATOflag = ATOflag;
    atoSettings.tATOmax = ATOmax;
    EEPROM_writeAnything(508, atoSettings);
  break; */
  case 8:                                   //save Moonlight settings
    EEPROM.write(517, 128);
    EEPROM_writeAnything(518, moonSett);
  break;
  case 9:                                   //save general settings
    EEPROM.write(539, 129);
    EEPROM.write(540, LCDbright);
  break;
  }
}

void ReadFromEEPROM()
{
  float t1;                                                //temporary value
  
  int k = EEPROM.read(0);                 //read saved values for channel 1
  delay(5);
  if (k==121) {
    for (int i=0; i<96; i++)  {
      ch1led[i] = EEPROM.read(i+1);
      delay(5);
    }
  }
  
  k = EEPROM.read(97);                      //read saved values for channel 2
  delay(5);
  if (k==122) {
    for (int i=0; i<96; i++)  {
      ch2led[i] = EEPROM.read(i+98);
      delay(5);
    }
  }
  
  k = EEPROM.read(194);                      //read saved values for channel 3
  delay(5);
  if (k==123) {
    for (int i=0; i<96; i++)  {
      ch3led[i] = EEPROM.read(i+195);
      delay(5);
    }
  }
  
  k = EEPROM.read(291);                      //read saved values for channel 4
  delay(5);
  if (k==124) {
    for (int i=0; i<96; i++)  {
      ch4led[i] = EEPROM.read(i+292);
      delay(5);
    }
  }

/*  k = EEPROM.read(388);                      //read saved values for channel 5
  delay(5);
  if (k==125) {
    for (int i=0; i<96; i++)  {
      ch5led[i] = EEPROM.read(i+389);
      delay(5);
    }
  } */
  
  k = EEPROM.read(485);                      //read temperature settings
  delay(5);
  if (k==126) {  
    EEPROM_readAnything(486, tempSettings);  
    t1 = tempSettings.tempset;
    tempVal.tempSet = t1/10;
    t1 = tempSettings.tempoff;
/*    tempVal.tempOff = t1/10;
    tempVal.tempLEDmin = tempSettings.tempLEDmin;
    tempVal.tempLEDmax = tempSettings.tempLEDmax;
    tempVal.tempLEDcut = tempSettings.tempLEDcut; */
  }
  
/*  k = EEPROM.read(507);  
  delay(5);
  if (k==127) {
    EEPROM_readAnything(508, atoSettings);
    ATOflag = atoSettings.tATOflag;
    ATOmax = atoSettings.tATOmax;
  } */
  
  k = EEPROM.read(517);
  delay(5);
  if (k==128) 
    EEPROM_readAnything(518, moonSett);
  
  k = EEPROM.read(539);
  delay(5);
  if (k==129) 
    LCDbright = EEPROM.read(540);
  
 //Serial.println(tempVal.tempLEDcut);
}

void ClearEEPROM()
{
  // write a 0 to all 4096 bytes of the EEPROM
  for (int i = 0; i < 4096; i++)
    EEPROM.write(i, 0);
}

//------------------------RTC-----------------------------
void SaveRTC()
  {        
   RTC.stopClock();     

   RTC.fillByYMD(tmpRTC.tYear, tmpRTC.tMonth, tmpRTC.tDay);
   RTC.fillByHMS(tmpRTC.tHour, tmpRTC.tMinute, 0);
   RTC.setTime();
   delay(10);
   
   RTC.startClock();
   delay(10);
  }


//---------------------------LED levels---------------------------
void LED_levels_output()
{
int sector, sstep, t1, t2 ;
int l_out;

if (LEDtestFlag==0)
  min_cnt= (RTC.hour*60)+RTC.minute;
 
 if (min_cnt>=1440) {min_cnt=1;}   // 24 hours of minutes 
 sector = min_cnt/15;              // divided by gives sector -- 15 minutes
 sstep = min_cnt%15;               // remainder gives add on to sector value 
 
  t1 =sector;
  if (t1==95) {t2=0;}
   else {t2 = t1+1;}
  
  for (int i=0; i<numberOfCh; i++)  
   {
    if (sstep==0) 
      LEDch_out[i] = LEDval_arr[i][t1];
    else 
      LEDch_out[i] = check(&LEDval_arr[i][t1], &LEDval_arr[i][t2], sstep); 
    
    LEDch_out[i] = (LEDch_out[i] * LEDcutOff) / 100;       
    
    if (BUCKPUCK) 
      l_out = LEDch_out[i];
    else 
      l_out = 100 - LEDch_out[i]; 
    
    l_out = map(l_out, 0, 100, 0, 255);    
    analogWrite(LEDpin[i], l_out);
  }
 
  //Serial.println(b_out);
  //Serial.println(w_out);
}

int check( byte *pt1, byte *pt2, int lstep)
{
  int result;
  float fresult;
   
  if (*pt1==*pt2) {result = *pt1;}      // No change
    else if (*pt1<*pt2)                 //Increasing brightness
    { fresult = ((float(*pt2-*pt1)/15.0) * float(lstep))+float(*pt1);
     result = int(fresult);
     }
     //Decreasing brightness
    else {fresult = -((float(*pt1-*pt2)/15.0) * float(lstep))+float(*pt1);
     result = int(fresult);                     
    } 
    return result;
}

//-----------------------------------Moonlight output-------------------------------
void moon_init()
{
  long tnow;
  char tmpChar[3];
  
  moonphase(RTC.year, RTC.month, RTC.day);
  
  if (moonSett.timesReal == 1) {
    riseset(latitude, longtitude);
    
    RTC.getTime();
    tnow = RTC.time2000;
    if (Moon.isRise) {
      RTC.fillByHMS(Moon.riseH, Moon.riseM, 0);
      rise_time = RTC.time2000;
    } else {
      RTC.fillByHMS(0, 0, 0);
      rise_time = RTC.time2000;
    }
    if (Moon.isSet) {
      RTC.fillByHMS(Moon.setH, Moon.setM, 0);
      set_time = RTC.time2000;
    } else {
      RTC.fillByHMS(23, 59, 59);
      set_time = RTC.time2000;
    }
    RTC.getTime();
    
    moonUp = 0;
    if (rise_time < set_time) {
      if ((tnow >=rise_time) && (tnow < set_time)) 
        moonUp = 1; }
    else if ((tnow >= rise_time) || (tnow < set_time)) {
      moonUp = 1; }
      
    riseSet_descr[0] = '\0';
    
    if (moonUp) {
      if ( (!Moon.isSet) || ((rise_time > set_time)&&(tnow>=rise_time)) ) {      //calculate for next day
        riseset(latitude, longtitude, 1);       
	RTC.fillByYMD(RTC.year, RTC.month, RTC.day+1);
      }
      strcat(riseSet_descr, "Moonset ");
      itoa(Moon.setH, tmpChar, 10);
      strcat(riseSet_descr, tmpChar);
      strcat(riseSet_descr, ":");
      if (Moon.setM < 10)
        strcat(riseSet_descr, "0");
      itoa(Moon.setM, tmpChar, 10);
      strcat(riseSet_descr, tmpChar);
      
      RTC.fillByHMS(Moon.setH, Moon.setM, 59);
      set_time = RTC.time2000;
    } 
    else {                                  
      if ( (!Moon.isRise) || ((set_time > rise_time)&&(tnow>=set_time)) ) {      //calculate for next day
        riseset(latitude, longtitude, 1);
	RTC.fillByYMD(RTC.year, RTC.month, RTC.day+1);
      }
      strcat(riseSet_descr, "Moonrise ");
      itoa(Moon.riseH, tmpChar, 10);
      strcat(riseSet_descr, tmpChar);
      strcat(riseSet_descr, ":");
      if (Moon.riseM < 10)
        strcat(riseSet_descr, "0");
      itoa(Moon.riseM, tmpChar, 10);
      strcat(riseSet_descr, tmpChar);
      
      //RiseSet_out = 1;
      RTC.fillByHMS(Moon.riseH, Moon.riseM, 0);
      rise_time = RTC.time2000;
    } 
  }
  else {                              //moonSett.timesReal == 0
    if (moonUp == 0) {
      if ((RTC.hour >= moonSett.moonrise) || (RTC.hour < moonSett.moonset)) {       
        moonUp = 1;  
      }
    } else {
      if ((RTC.hour < moonSett.moonrise) && (RTC.hour >= moonSett.moonset)) {        
        moonUp = 0;
      }
    }
  }
  
  RTC.getTime();
  
  //  Serial.println(moonUp);
}

void moonlight_output()
{
  int tout;
  byte mout;
  long tnow = RTC.time2000;
  
  if (moonUp == 1) {
    tout = map(Moon.light, 0, 100, moonSett.newMoonLight, moonSett.fullMoonLight);
    tout = map(tout, 0, 100, 0, 255); 
    mout = byte(tout);   
   
    if (moonSett.timesReal == 0) {                               //simple mode
      if ((moonSett.moonrise==RTC.hour) && (RTC.minute<=15)) {   //dimm for moonrise
	mout = check(0, &mout, RTC.minute);
      }
      else if ((moonSett.moonset-1==RTC.hour) && (RTC.minute>=45)) {   //dimm for moonset
        mout = check(&mout, 0, RTC.minute-45);
      }
      else if (moonSett.moonset==RTC.hour) {
        mout = 0;
      }
    }
    else {                                                       //precise mode
      if ((tnow>=rise_time) && (tnow<=rise_time+900)) {                  //dimm for moonrise
	mout = check(0, &mout, (tnow-rise_time)/60);
      }
      else if ((tnow>=set_time-900) && (tnow<=set_time)) {               //dimm for moonset
	mout = check(&mout, 0, (tnow-(set_time-900))/60);
      } 
    }
    analogWrite(moonlightPin, mout);
    
    //Serial.println(mout);
  } 
  else 
    digitalWrite(moonlightPin, LOW);

}

////////////////////////////////////////////////////////////////////////////////////////////
//                                               SCREENS
////////////////////////////////////////////////////////////////////////////////////////////

//---------------------------------------main screen ----------dispScreen = 0
void mainScreen(boolean refreshAll=false)
{
  int t1, by, bary;
  char tmpChar[5];

  if (numberOfCh < 4)
    by = 110;
  else 
    by = 124;
                           
  if ( refreshAll)                                    //draw static elements
    {
     myGLCD.setColor(0, 0, 125);
     myGLCD.drawLine(6, 24, 313, 24);                 //top line
     myGLCD.drawLine(6, 25, 313, 25); 
     myGLCD.drawLine(6, by-1, 313, by-1);             //middle line
     myGLCD.drawLine(6, by, 313, by); 
     myGLCD.drawLine(180, by+4, 180, 222);            //vertical line x=194
       
     changeFont(SMALL, 255, 255, 255, 0, 0, 0);
     myGLCD.print("Water", 20, 130);
     myGLCD.print("temp:", 20, 141);
     myGLCD.print("LEDs temp:", 20, 165);
     myGLCD.drawCircle(150, 167, 1);
     myGLCD.print("C", 153, 165);
     myGLCD.print("Fans speed:", 20, 178);
    }
  
  if ((RTC.hour!=prevRTC.tHour) || (RTC.minute!=prevRTC.tMinute) || refreshAll) {    //time
    prevRTC.tHour = RTC.hour;
    prevRTC.tMinute = RTC.minute;
    changeFont(LARGE, 225, 225, 0, 0, 0, 0);
    printTime(RTC.hour, RTC.minute, 220, 5);
  }

  if ((RTC.day!=prevRTC.tDay) || (RTC.month!=prevRTC.tMonth) || (RTC.year!=prevRTC.tYear) || refreshAll) {     //date
    prevRTC.tDay = RTC.day;
    prevRTC.tMonth = RTC.month;
    prevRTC.tYear = RTC.year;
    changeFont(SMALL, 240, 240, 255, 0, 0, 0);
    printDate(15, 8);             
  }
  
  for (int i=0; i<numberOfCh; i++)  {                                  //led bars
    if ((prevLEDch_out[i] != LEDch_out[i]) || refreshAll) {      
      bary = 25+ ((by-15)/(numberOfCh+1))*(i+1) -12;
      drawLedBar(bary, i+1);  
      prevLEDch_out[i] = LEDch_out[i];  }
  }
    
/*  myGLCD.setColor(0, 0, 0);                                //clear alarm notice
  myGLCD.fillRect(alaR[0], alaR[1], alaR[2], alaR[3]+1);  
  
  if (ATOflag == 3) {                                      //ato notice
    changeFont(LARGE, 255, 0, 0, 0, 0, 0);
    myGLCD.drawRect(alaR[0], alaR[1], alaR[2], alaR[3]);
    myGLCD.print("ATO fail!", 15, 199);    
  }
  
  t1 = map(Fan1PWMval, 0, 639, 0, 100);                    //fans speed
  tmpChar[0] = '\0';
  itoa(t1, tmpChar, 10);
  strcat(tmpChar, "%  ");
  changeFont(SMALL, 255, 255, 255, 0, 0, 0);  
  myGLCD.print(tmpChar, 176-strlen(tmpChar)*8, 178);      // fan speed 
  if (!LEDcutFlag)
    myGLCD.printNumF(tempH, 1, 113, 165);                 // heatsink temperature in white
  else {
    myGLCD.setColor(255, 255, 0);
    myGLCD.printNumF(tempH, 1, 113, 165);                 // heatsink temperature in yellow
    changeFont(SMALL, 127, 0, 0, 0, 0, 0);
    myGLCD.drawRect(alaR[0], alaR[1], alaR[2], alaR[3]);
    myGLCD.print("ackn", 135, 207);
  }
*/  
  if (tempAlarmFlag == 1) {                                //print water temperature
    changeFont(LARGE, 255, 0, 0, 0, 0, 0);
    myGLCD.drawRect(alaR[0], alaR[1], alaR[2], alaR[3]);
    myGLCD.printNumF( tempC, 1, 74, 135);                  //temperature in red
    myGLCD.drawCircle(142, 139, 2);    
    myGLCD.print("C", 145, 137);      
    myGLCD.print("ALARM!!", 15, 199);
/*    if (AlarmSoundFlag==true) {                              //print acknowledge
      changeFont(SMALL, 127, 0, 0, 0, 0, 0);
      myGLCD.print("ackn", 135, 207);
    } */
  } else if (tempAlarmFlag == 2) {
    changeFont(LARGE, 255, 255, 0, 0, 0, 0);
    myGLCD.printNumF( tempC, 1, 74, 135);                  //temperature in yellow
    myGLCD.drawCircle(142, 139, 2);    
    myGLCD.print("C", 145, 135);  
    changeFont(SMALL, 127, 0, 0, 0, 0, 0);
    myGLCD.drawRect(alaR[0], alaR[1], alaR[2], alaR[3]);
    myGLCD.print("ackn", 135, 207);      
  }  
  else {
    changeFont(LARGE, 0, 255, 0, 0, 0, 0);
    myGLCD.printNumF( tempC, 1, 74, 135);                // water temperature
    myGLCD.drawCircle(142, 139, 2);    
    myGLCD.print("C", 145, 135);
  }
    
  if ((tmpMoon.light!=Moon.light) || refreshAll) {       //moonlight part
    tmpMoon = Moon;
    tmpChar[0] = '\0';                                         // moon
    itoa(Moon.light, tmpChar, 10);
    strcat(tmpChar, "%");  
    changeFont(SMALL, 255, 255, 255, 0, 0, 0);
    myGLCD.print(tmpChar, 251-stringCenter(tmpChar), 186); 
    myGLCD.print(MPhase[Moon.phase], 251-stringCenter(MPhase[Moon.phase]), 198);
    if (moonSett.timesReal)
      myGLCD.print(riseSet_descr, 251-stringCenter(riseSet_descr), 210);
    myGLCD.drawBitmap(226, by+(186-by)/2-24, 50, 50, MoonPics[Moon.phase-1]);   
  }
 
//  Serial.println();
}

//------------------------------------menu screen------- dispScreen=1 -------------------
void menuScreen()
{
  printHeader("Choose option:");
      
  myGLCD.setColor(0, 0, 125);
  myGLCD.drawLine(6, 20, 313, 20);
  myGLCD.drawLine(159, 24, 159, 56);
  myGLCD.drawLine(6, 60, 313, 60);
  myGLCD.drawLine(159, 64, 159, 96);
  myGLCD.drawLine(6, 100, 313, 100);
  myGLCD.drawLine(159, 104, 159, 136);
  myGLCD.drawLine(6, 140, 313, 140);
  myGLCD.drawLine(159, 144, 159, 176);
  myGLCD.drawLine(6, 180, 313, 180);
  
  myGLCD.drawLine(6, 192, 313, 192);
  myGLCD.drawLine(6, 193, 313, 193);
  myGLCD.drawLine(159, 197, 159, 221);
  
  printButton("TIME and DATE", tanD[0], tanD[1], tanD[2], tanD[3]);
  printButton("TEMP. SETTINGS", temC[0], temC[1], temC[2], temC[3]);
//  printButton("ATO", atoS[0], atoS[1], atoS[2], atoS[3]);
  printButton("SETTINGS", scrS[0], scrS[1], scrS[2], scrS[3]);
  printButton("LEDs OUTPUT", ledW[0], ledW[1], ledW[2], ledW[3]);
  printButton("TEST LEDs", tesT[0], tesT[1], tesT[2], tesT[3]);
  printButton("COLOR MIXER", colM[0], colM[1], colM[2], colM[3]);
  printButton("CANCEL", canC[0], canC[1], canC[2], canC[3]);
}

//-----------------------------------set clock screen-------dispScreen=2 ---------------
void clockScreen(boolean refreshAll=false)
{
    char char1[4], tmp[3];
  
    if (refreshAll)
    {
      tmpRTC.tHour = RTC.hour;
      tmpRTC.tMinute = RTC.minute;
      tmpRTC.tDow = RTC.dow;
      tmpRTC.tDay = RTC.day;
      tmpRTC.tMonth = RTC.month;
      tmpRTC.tYear = RTC.year;
      
      printHeader("Time and Date settings");
      
      myGLCD.setColor(0, 0, 125);
      myGLCD.drawLine(6, 103, 313, 103);

      changeFont(LARGE, 255, 255, 0, 0, 0, 0);
      myGLCD.print("Time:", 20, 51);
      myGLCD.print("Date:", 20, 140);
      
      printUpButton(houP[0], houP[1]);            //hour plus
      printDownButton(houM[0], houM[1]);          //hour minus
      printUpButton(minP[0], minP[1]);            //minutes plus
      printDownButton(minM[0], minM[1]);          //minutes minus
      
      printUpButton(dayU[0], dayU[1]);            //
      printDownButton(dayD[0], dayD[1]);
      printUpButton(monU[0], monU[1]);
      printDownButton(monD[0], monD[1]);
      printUpButton(yeaU[0], yeaU[1]);
      printDownButton(yeaD[0], yeaD[1]);
      
      myGLCD.setColor(0, 0, 125);
      myGLCD.drawLine(6, 192, 313, 192);
      myGLCD.drawLine(6, 193, 313, 193);
      myGLCD.drawLine(159, 197, 159, 221);
      printButton("SAVE", prOK[0], prOK[1], prOK[2], prOK[3]);
      printButton("CANCEL", canC[0], canC[1], canC[2], canC[3]);
    }
    
    changeFont(LARGE, 255, 255, 255, 0, 0, 0);
    
    char1[0] = '\0';
    itoa(tmpRTC.tHour, char1, 10);                        //print hour
    myGLCD.print("   ", houP[0]-5, 51);
    myGLCD.print(char1, houP[0]+14-stringCenter(char1, LARGE), 51);
    myGLCD.print(":", 168, 51);
    
    char1[0] = '\0';
    if (tmpRTC.tMinute <= 9) {                            //print minutes
      strcat(char1, "0");
      itoa(tmpRTC.tMinute, tmp, 10);
      strcat(char1, tmp);
    }
    else 
      itoa(tmpRTC.tMinute, char1, 10);
    myGLCD.print("   ", minP[0]-2, 51);
    myGLCD.print(char1, minP[0]+14-stringCenter(char1, LARGE), 51);
    
    char1[0] = '\0';
    itoa(tmpRTC.tDay, char1, 10);                        //print day
    myGLCD.print("   ", dayU[0]-5, 140);
    myGLCD.print(char1, dayU[0]+14-stringCenter(char1, LARGE), 140);
    
    myGLCD.print(Mon[tmpRTC.tMonth], monU[0]+14-stringCenter(Mon[tmpRTC.tMonth], LARGE), 140);    //print month
    
    char1[0] = '\0';
    itoa(tmpRTC.tYear, char1, 10);                                                 //print year
    myGLCD.printNumI(tmpRTC.tYear, yeaU[0]+14-stringCenter(char1, LARGE), 140);             
}

//--------------------------------------LED test screen---------dispScreen=3 --------------------
void testScreen(boolean refreshAll=false)
{    
  int bary;
  
  if (refreshAll) 
   {  
    printHeader("Test LEDs output settings");
    myGLCD.setColor(0, 0, 0);
    myGLCD.fillRect (2, 17, 317, 155);                        //clear 'Test in progress'
    myGLCD.fillRect(stsT[0], stsT[1], stsT[2], stsT[3]);
     
    printButton ("START TEST", stsT[0], stsT[1], stsT[2], stsT[3]);   //start
    printButton ("-10s", tenM[0], tenM[1], tenM[2], tenM[3]);         //-10s
    printButton ("+10s", tenP[0], tenP[1], tenP[2], tenP[3]);         //+10s  

    myGLCD.setColor(0, 0, 125);
    myGLCD.drawLine(6, 157, 313, 157);
    myGLCD.drawLine(75, 161, 75, 188);
    myGLCD.drawLine(244, 161, 244, 188);
    myGLCD.drawLine(6, 192, 313, 192);
    myGLCD.drawLine(6, 193, 313, 193);
    myGLCD.drawLine(159, 197, 159, 221);
    printButton("RESET", prOK[0], prOK[1], prOK[2], prOK[3]);
    printButton("CANCEL", canC[0], canC[1], canC[2], canC[3]);
  } 

   if (LEDtestFlag==1) {     
     myGLCD.setColor(255, 0, 0);
     myGLCD.fillRect (stsT[0]+4, stsT[1]+4, stsT[2]-4, stsT[3]-4);       //red button during test
     changeFont(SMALL, 255, 255, 255, 255, 0, 0);
     myGLCD.print("STOP TEST", 123, 170); 
     changeFont(LARGE, 255, 255, 255, 0, 0, 0);
     myGLCD.print("Time:", 70, 135);     
   
     while (LEDtestFlag>0)                                //test LED and speed up time
     {
      unsigned long cMillis = millis();
       
      if (myTouch.dataAvailable())  {
        processMyTouch();
      }
     
      if ((cMillis - prevMillis05 > 500) && (LEDtestFlag == 1))   //change time every 0.5s
        {
         prevMillis05 = cMillis;

         min_cnt++;
         if (min_cnt > 1440)
           min_cnt = 0;
         int tmphour = min_cnt/60;
         int tmpminut = min_cnt%60;
         changeFont(LARGE, 255, 255, 0, 0, 0, 0);
         printTime(tmphour, tmpminut, 162, 135);
         
         for (int i=0; i<numberOfCh; i++)  {
           if ((prevLEDch_out[i] != LEDch_out[i]) || refreshAll) {                          //
           bary = 0+(130/(numberOfCh+1))*(i+1);
           drawLedBar(bary, i+1);  
           prevLEDch_out[i] = LEDch_out[i];  }
         }
         
         LED_levels_output();
    
         }
      }
    } 
}

//--------------------------------------------- Temperature Settings screen ------dispScreen=4 ------------
void tempScreen(boolean refreshAll=false)
{
  if (refreshAll) {
    tmpTempVal = tempVal;
     
    printHeader("Temperature Settings");
     
    myGLCD.setColor(0, 0, 125);
    myGLCD.drawLine(6, 23, 313, 23);
    myGLCD.drawLine(6, 81, 313, 81);
    myGLCD.drawLine(6, 140, 313, 140);
  
    myGLCD.drawLine(6, 192, 313, 192);
    myGLCD.drawLine(6, 193, 313, 193);
    myGLCD.drawLine(159, 197, 159, 221);
    printButton("SAVE", prOK[0], prOK[1], prOK[2], prOK[3]);
    printButton("CANCEL", canC[0], canC[1], canC[2], canC[3]);
     
    changeFont(SMALL, 255, 255, 0, 0, 0, 0);
    myGLCD.print("Water Temperature Alarm:", 15, 18);
    myGLCD.print("LEDs temp. when fans:", 15, 76);
    myGLCD.print("LEDs temp. safety cut off:", 15, 135);
    changeFont(SMALL, 170, 170, 255, 0, 0, 0);
    myGLCD.print("Optimal", 7, 37);
    myGLCD.print("temp.:", 7, 51);
    myGLCD.print("+-:", 193, 45); 
    myGLCD.print("Starts:", 17, 103);
    myGLCD.print("100%:", 176, 103);
     
    printUpButton(temP[0], temP[1]);
    printDownButton(temM[0], temM[1]);
    printUpButton(offP[0], offP[1]);
    printDownButton(offM[0], offM[1]);
     
    printUpButton(hemP[0], hemP[1]);
    printDownButton(hemM[0], hemM[1]);
    printUpButton(hetP[0], hetP[1]);
    printDownButton(hetM[0], hetM[1]);
     
    printUpButton(hecP[0], hecP[1]);
    printDownButton(hecM[0], hecM[1]);
   }
    
    changeFont(LARGE, 255, 255, 255, 0, 0, 0);
    myGLCD.printNumF(tmpTempVal.tempSet, 1, 69, 43);
    if (tmpTempVal.tempOff <0.1)
       myGLCD.print("OFF", 222, 43);
    else
       myGLCD.printNumF(tmpTempVal.tempOff, 1, 222, 43);
    myGLCD.printNumI(tmpTempVal.tempLEDmin, 78, 102);
    myGLCD.printNumI(tmpTempVal.tempLEDmax, 218, 102);
    if (tmpTempVal.tempLEDcut == 0)
       myGLCD.print("OFF", 104, 161);
    else {
       myGLCD.print(" ", 104, 161);
       myGLCD.printNumI(tmpTempVal.tempLEDcut, 120, 161); 
    }

   //Serial.println(tmpTempVal.tempOff);
}

//------------------------------------LEDs menu screen------- dispScreen=5 -------------------
void LEDmenuScreen()
{  
  printHeader("Select LED channel:");
    
  myGLCD.setColor(0, 0, 125);
  myGLCD.drawLine(6, 20, 313, 20);
  myGLCD.drawLine(159, 24, 159, 56);
  myGLCD.drawLine(6, 60, 313, 60);
  myGLCD.drawLine(159, 64, 159, 96);
  myGLCD.drawLine(6, 100, 313, 100);
  myGLCD.drawLine(159, 104, 159, 136);
  myGLCD.drawLine(6, 140, 313, 140);
  myGLCD.drawLine(159, 144, 159, 176);
  myGLCD.drawLine(6, 180, 313, 180);
      
  myGLCD.drawLine(6, 192, 313, 192);
  myGLCD.drawLine(6, 193, 313, 193);
  myGLCD.drawLine(159, 197, 159, 221);
     
  printButton(ch1Descr, ch1L[0], ch1L[1], ch1L[2], ch1L[3]);
  printButton(ch2Descr, ch2L[0], ch2L[1], ch2L[2], ch2L[3]);
  
  if (numberOfCh>2) 
    printButton(ch3Descr, ch3L[0], ch3L[1], ch3L[2], ch3L[3]);
  
  if (numberOfCh>3) 
    printButton(ch4Descr, ch4L[0], ch4L[1], ch4L[2], ch4L[3]);
  
  if (numberOfCh>4) 
    printButton(ch5Descr, ch5L[0], ch5L[1], ch5L[2], ch5L[3]);
      
  printButton("MOONLIGHTS", mooL[0], mooL[1], mooL[2], mooL[3]);
  
  printButton("CANCEL", canC[0], canC[1], canC[2], canC[3]);
}

void ledSetScreen()   //-------------------------show all led values screen, dispScreen==6
{
  int a;  
  
  tChar30[0] = '\0';
  
  switch (selectChannel) {
    case 1:
      for (int i=0; i<96; i++)
        tled[i] = ch1led[i];
      strcat(tChar30, ch1Descr);
      strcat(tChar30, " output values %" );
      printHeader(tChar30);
    break;
    case 2:
      for (int i; i<96; i++)
        tled[i] = ch2led[i];
      strcat(tChar30, ch2Descr);
      strcat(tChar30, " output values %" );
      printHeader(tChar30);
    break;
    case 3:
      for (int i; i<96; i++)
        tled[i] = ch3led[i];
      strcat(tChar30, ch3Descr);
      strcat(tChar30, " output values %" );
      printHeader(tChar30);
    break;
    case 4:
      for (int i; i<96; i++)
        tled[i] = ch4led[i];
      strcat(tChar30, ch4Descr);
      strcat(tChar30, " output values %" );
      printHeader(tChar30);
    break;
    case 5:
      for (int i; i<96; i++)
        tled[i] = ch5led[i];
      strcat(tChar30, ch5Descr);
      strcat(tChar30, " output values %" );
      printHeader(tChar30);
    break;
  }
  
  changeFont(SMALL, 255, 255, 255, 0, 0, 0);
  for (int i=0; i<12; i++) {
    myGLCD.setColor(255, 255, 0);
    myGLCD.printNumI((i*2), (i*26)+19-(intlen(i*2)*4), 18);
    myGLCD.printNumI(((i*2)+1), (i*26)+19-(intlen(i*2)*4), 28);
     for (int j=0; j<8; j++) {
        a= (i*8)+j;         
        myGLCD.setColor(255, 255, 255);
        myGLCD.printNumI(tled[a], (i*26)+19-(intlen(tled[a])*4), (j*18)+44);   
        myGLCD.setColor(100, 100, 100);
        myGLCD.drawRect((i*26)+4, (j*18)+40, (i*26)+30, (j*18)+58);
    }
  }
  
  myGLCD.setColor(0, 0, 125);  
  myGLCD.drawLine(6, 192, 313, 192);
  myGLCD.drawLine(6, 193, 313, 193);
  myGLCD.drawLine(159, 197, 159, 221);
  printButton("CHANGE", prOK[0], prOK[1], prOK[2], prOK[3]);
  printButton("BACK", canC[0], canC[1], canC[2], canC[3]);
}

void ledChangeScreen()    //-------------------------change led values screen, dispScreen=7 -----------
{
  printHeader(tChar30);
     
  changeFont(SMALL, 0, 255, 255, 0,0,0);
  for (int i=0; i<12; i++) {
    myGLCD.setColor(255, 255, 0);
    myGLCD.printNumI((i*2), (i*26)+19-(intlen(i*2)*4), 20);
    myGLCD.printNumI(((i*2)+1), (i*26)+19-(intlen(i*2)*4), 31);
    myGLCD.setColor(100, 100, 100);
    myGLCD.drawRect((i*26)+4, 19 , (i*26)+30, 43);
    }
  
  myGLCD.setColor(0, 0, 125);  
  myGLCD.drawLine(6, 192, 313, 192);
  myGLCD.drawLine(6, 193, 313, 193);
  myGLCD.drawLine(159, 197, 159, 221);
  printButton("SAVE", prOK[0], prOK[1], prOK[2], prOK[3]);
  printButton("CANCEL", canC[0], canC[1], canC[2], canC[3]);
}

//---------------------------------------------ATO settings screen, dispScreen=8 ------------------------
void ATOScreen(boolean refreshAll=false)    
{
  char tmpChar[4];
  
  if (refreshAll) 
    {
     tmpATOflag = ATOflag;
     tmpATOmax = ATOmax;
      
     printHeader("ATO Settings");
  
     changeFont(SMALL, 255, 255, 255, 0, 0, 0);
     myGLCD.print("Turn on ATO:", 35, 37);
     if (ATOflag == 3) {
       changeFont(SMALL, 255, 0, 0, 0, 0, 0);
       myGLCD.print("ATO failure", 35, 53);
       }
     
     myGLCD.setColor(0, 0, 125);
     myGLCD.drawLine(4, 80, 314, 80);
     myGLCD.drawLine(4, 150, 314, 150);

     changeFont(SMALL, 255, 255, 255, 0, 0, 0);     
     myGLCD.print("Max time for", 18, 104);
     myGLCD.print("ATO pump (s):", 18, 117);
     
     printUpButton(atoP[0], atoP[1]);
     printDownButton(atoM[0], atoM[1]);
     
     myGLCD.setColor(0, 0, 125);
     myGLCD.drawLine(6, 192, 313, 192);
     myGLCD.drawLine(6, 193, 313, 193);
     myGLCD.drawLine(159, 197, 159, 221);
     printButton("SAVE", prOK[0], prOK[1], prOK[2], prOK[3]);
     printButton("CANCEL", canC[0], canC[1], canC[2], canC[3]);
    }
    
  if ((tmpATOflag == 1) || (tmpATOflag == 2))                     //print on switch
    printOnOffbutton(1, atoO[0], atoO[1]);
  else 
    printOnOffbutton(0, atoO[0], atoO[1]);                          //print off switch
    
  if (tmpATOflag != 3) {
      changeFont(SMALL, 255, 0, 0, 0, 0, 0);
      myGLCD.print("            ", 35, 53);
    }  
  
  changeFont(LARGE, 255, 255, 255, 0, 0, 0);
  myGLCD.print(" ", 137, 108);
  myGLCD.printNumI(tmpATOmax, 169-intlen(tmpATOmax)*16, 108); 
  
}

//--------------------------------------------- Moonlights settings screen, dispScreen=9 ------------------------
void moonScreen(boolean refreshAll=false)    
{  
  if (refreshAll) {
    tmpMoonSett = moonSett;
    
    printHeader("Moonlight Settings");
    
    myGLCD.setColor(0, 0, 125);
    myGLCD.drawLine(6, 66, 313, 66);
    myGLCD.drawLine(6, 131, 313, 131);    
    
    changeFont(SMALL, 170, 170, 255, 0, 0, 0);
    myGLCD.print("Use real moonrise", 17, 25);
    myGLCD.print("and moonset times:", 17, 37);
    myGLCD.print("Moon", 20, 85);
    myGLCD.print("rise:", 20, 97);
    myGLCD.print("Moon", 180, 85);
    myGLCD.print("set:", 180, 97);
    myGLCD.print("New", 20, 153);
    myGLCD.print("Moon:", 20, 165);
    myGLCD.print("Full", 180, 153);
    myGLCD.print("Moon:", 180, 165);
    
    changeFont(SMALL, 255, 255, 0, 0, 0, 0);
    myGLCD.print("Set times if above is OFF:", 10, 61);
    myGLCD.print("Moon ilumination (%) during:", 10, 126);
  
    printUpButton(morP[0], morP[1]);      //moonrise time plus
    printDownButton(morM[0], morM[1]);      //moonrise time minus
    printUpButton(mosP[0], mosP[1]);      //moonset time plus
    printDownButton(mosM[0], mosM[1]);      //moonset time minus
    
    printUpButton(nmlP[0], nmlP[1]);      //new moon light value plus
    printDownButton(nmlM[0], nmlM[1]);      //new moon light value minus
    printUpButton(fmlP[0], fmlP[1]);      //full moon light value plus
    printDownButton(fmlM[0], fmlM[1]);      //full moon light value minus
    
  
    myGLCD.setColor(0, 0, 125);  
    myGLCD.drawLine(6, 192, 313, 192);
    myGLCD.drawLine(6, 193, 313, 193);
    myGLCD.drawLine(159, 197, 159, 221);
    printButton("SAVE", prOK[0], prOK[1], prOK[2], prOK[3]);
    printButton("BACK", canC[0], canC[1], canC[2], canC[3]); 
  }
  
  printOnOffbutton(tmpMoonSett.timesReal, morO[0], morO[1]);
  
  if (tmpMoonSett.timesReal)
    changeFont(LARGE, 128, 128, 128, 0, 0, 0);
  else 
    changeFont(LARGE, 255, 255, 255, 0, 0, 0);
  myGLCD.print(" ", 76, 90);
  myGLCD.printNumI(tmpMoonSett.moonrise, 108-intlen(tmpMoonSett.moonrise)*16, 90); 
  myGLCD.print(" ", 226, 90);
  myGLCD.printNumI(tmpMoonSett.moonset, 258-intlen(tmpMoonSett.moonset)*16, 90);   
  changeFont(LARGE, 255, 255, 255, 0, 0, 0);
  myGLCD.print("   ", 64, 158);
  myGLCD.printNumI(tmpMoonSett.newMoonLight, 110-intlen(tmpMoonSett.newMoonLight)*16, 158);
  myGLCD.print("   ", 218, 158);
  myGLCD.printNumI(tmpMoonSett.fullMoonLight, 264-intlen(tmpMoonSett.fullMoonLight)*16, 158);
}

//--------------------------------------------general settings screen, dispScreen=10 ------------------------
void genSetScreen(boolean refreshAll=false)    
{  
 byte drgb[]={0,0,125};
  
 if (refreshAll) { 
  tmpLCDbright = LCDbright;
  
  printHeader("General Settings");
  
  myGLCD.setColor(0, 0, 125);
  myGLCD.drawLine(159, 19, 159, 188);
  
  myGLCD.drawLine(163, 59, 313, 59);
  myGLCD.drawLine(163, 91, 313, 91);
  myGLCD.drawLine(163, 149, 313, 149);
  myGLCD.drawLine(163, 181, 313, 181);
  printButton("SAVE", gseS[0], gseS[1], gseS[2], gseS[3]);
  printButton("LOAD", gseL[0], gseL[1], gseL[2], gseL[3]);
  
  changeFont(SMALL, 255, 255, 255, 0, 0, 0);
  myGLCD.print("Screen brightness:", 10, 20);
  myGLCD.print("Save settings to", 170, 28);
  myGLCD.print("EEPROM memory:", 170, 40);
  myGLCD.print("Load settings from", 170, 120);
  myGLCD.print("EEPROM memory:", 170, 132);
  
  myGLCD.setColor(0, 0, 125);
  myGLCD.drawLine(6, 192, 313, 192);
  myGLCD.drawLine(6, 193, 313, 193);
  myGLCD.drawLine(159, 197, 159, 221);
  printButton("SAVE", prOK[0], prOK[1], prOK[2], prOK[3]);
  printButton("CANCEL", canC[0], canC[1], canC[2], canC[3]);
 }
 
 printVBar(tmpLCDbright, gseB[0], gseB[1], drgb);
}

//--------------------------------------------- Color Mixer screen, dispScreen=11 ---------------------------
void colorMixerScreen(boolean refreshAll=false)
{
  int posx;
  byte currentLedLevels[5];
  
  if (refreshAll) {
    for (int i=0; i<5; i++) 
      currentLedLevels[i] = LEDch_out[i];
    
    printHeader("LEDs Color Mixer");

    myGLCD.setColor(0, 0, 125);
    myGLCD.drawLine(6, 192, 313, 192);
    myGLCD.drawLine(6, 193, 313, 193);
    myGLCD.drawLine(159, 197, 159, 221);
    printButton("CANCEL", canC[0], canC[1], canC[2], canC[3]);
  }
  
  for (int i=0; i<numberOfCh; i++)
    {
     posx = i*(270/numberOfCh) + ((318/numberOfCh)/2+2);
     printVBar(currentLedLevels[i], posx, 22, rgbChann[i]);
     changeFont(LARGE, rgbChann[i][0], rgbChann[i][1], rgbChann[i][2], 0, 0, 0);
     myGLCD.print(namesCh[i+1], posx, 174);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                          PROCCESS TOUCH
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void processMyTouch()
{
  myTouch.read();
    x=myTouch.getX();
    y=myTouch.getY();
    
    prevMillis600 = millis();
    
    if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3]) && (dispScreen!=0) && (dispScreen!=6) && (dispScreen!=7) && (dispScreen!=9))  //press cancel
      {
       hlightButton("CANCEL", canC[0], canC[1], canC[2], canC[3]);
       LEDtestFlag = 0;
       byte bout = map(LCDbright, 0, 100, 0, 255);
       analogWrite(LCDbrightPin, bout);
       dispScreen=0;
       clearScreen();
       mainScreen(true);
     } else 
    {
      
    switch (dispScreen) {
    case 0:                       //---------------------- main screen -----------------------------------
      if ((x>=alaR[0]) && (x<=alaR[2]) && (y>=alaR[1]) && (y<=alaR[3]) /*&& (AlarmSoundFlag==1) */) {   //acknowledge alarm
       /* if (AlarmSoundFlag == 1) {
          AlarmSoundFlag = 0;
          digitalWrite(AlarmBuzzPin, LOW); 
        }
        else */ if (tempAlarmFlag == 2)
          tempAlarmFlag = 0;
/*        else if (LEDcutFlag)
          LEDcutFlag = 0;
*/          
        clearScreen();
        mainScreen(true);
      }
      else {                 // press anywhere
        dispScreen=1;
        clearScreen();
        menuScreen();
      }
      break;
      
    case 1:                        //------------------------------menu screen-------------------------------------
       
       
       if ((x>=tanD[0]) && (x<=tanD[2]))               //first column
         {
         if ((y>=tanD[1]) && (y<=tanD[3]))                      //press time and date settings
           {
            hlightButton("TIME and DATE", tanD[0], tanD[1], tanD[2], tanD[3]);
            dispScreen=2;
            clearScreen();
            clockScreen(true);
           } 
         if ((y>=temC[1]) && (y<=temC[3]))                      //press Temp control
           {
            hlightButton("TEMP. SETTINGS", temC[0], temC[1], temC[2], temC[3]);
            dispScreen=4;
            clearScreen();
            tempScreen(true); 
            }
/*          if ((y>=atoS[1]) && (y<=atoS[3]))                      //press ATO settings
           {
            hlightButton("ATO", atoS[0], atoS[1], atoS[2], atoS[3]);
            dispScreen=8;
            clearScreen();
            ATOScreen(true); 
            }  */
          if ((y>=scrS[1]) && (y<=scrS[3]))                      //press screen settings
           {
            hlightButton("SETTINGS", scrS[0], scrS[1], scrS[2], scrS[3]);
            dispScreen=10;
            clearScreen();
            genSetScreen(true); 
            } 
         }
        if ((x>=ledW[0]) && (x<=ledW[2]))                    //second column
          {
           if  ((y>=ledW[1]) && (y<=ledW[3]))                       //press LED settings
             {
              hlightButton("LEDs OUTPUT", ledW[0], ledW[1], ledW[2], ledW[3]);
              dispScreen=5;
              clearScreen();
              LEDmenuScreen(); 
              } 
         if ((y>=tesT[1]) && (y<=tesT[3]))                             //press LED test
           {
            hlightButton("TEST LEDs", tesT[0], tesT[1], tesT[2], tesT[3]);
            dispScreen=3;
            clearScreen();
            testScreen(true);
           } 
         if ((y>=colM[1]) && (y<=colM[3]))                             //press LEDs color mixer
           {
            hlightButton("COLOR MIXER", colM[0], colM[1], colM[2], colM[3]);
            dispScreen=11;
            clearScreen();
            colorMixerScreen(true); 
            }                
          }
      break;
      
    case 2:                      //-------------------------------------clock setup--------------------------
      if ((x>=prOK[0]) && (x<=prOK[2]) && (y>=prOK[1]) && (y<=prOK[3]))      //press ok
        {
        hlightButton("SAVE", prOK[0], prOK[1], prOK[2], prOK[3]);    
        SaveRTC();        
        moon_init();
        moonlight_output();
        dispScreen=0;
        clearScreen();
        mainScreen(true);
      }
      else {
      if ((y>=houP[1]) && (y<=houP[1]+23)) {                    //first row
        if ((x>=houP[0]) && (x<=houP[0]+29)) {                 //hour plus
           hlightUpButton(houP[0], houP[1]);
           tmpRTC.tHour++;
           if (tmpRTC.tHour>=24) {tmpRTC.tHour = 0; }
        }
        if ((x>=minP[0]) && (x<=minP[0]+29)) {                 //minute plus
           hlightUpButton(minP[0], minP[1]);
           tmpRTC.tMinute++;
             if (tmpRTC.tMinute>=60) {tmpRTC.tMinute = 0; }
        }
      }
      else if ((y>=houM[1]) && (y<=houM[1]+23)) {                    //second row   
        if ((x>=houM[0]) && (x<=houM[0]+29)) {                    //hour minus
           hlightDownButton(houM[0], houM[1]);
           tmpRTC.tHour--;
           if (tmpRTC.tHour<0) {tmpRTC.tHour = 23; } 
        }        
        if ((x>=minM[0]) && (x<=minM[0]+29)) {                    //minute minus
           hlightDownButton(minM[0], minM[1]);
           tmpRTC.tMinute--;
           if (tmpRTC.tMinute<0) {tmpRTC.tMinute = 59; } 
        }
      }
      else if ((y>=dayU[1]) && (y<=dayU[1]+23)) {                      //third row
        if ((x>=dayU[0]) && (x<=dayU[0]+29)) {                    //press day up
           hlightUpButton(dayU[0], dayU[1]);
           tmpRTC.tDay++;
           if (tmpRTC.tDay>31) {tmpRTC.tDay = 1; }
        }
        if ((x>=monU[0]) && (x<=monU[0]+29)) {                    //press month up
           hlightUpButton(monU[0], monU[1]);
           tmpRTC.tMonth++;
           if (tmpRTC.tMonth>12) {tmpRTC.tMonth = 1; }
        }
        if ((x>=yeaU[0]) && (x<=yeaU[0]+29)) {                    //press year up
           hlightUpButton(yeaU[0], yeaU[1]);
           if (tmpRTC.tYear<2100) 
             tmpRTC.tYear++;
        } 
      }
      else if ((y>=dayD[1]) && (y<=dayD[1]+23))  {                        //fourth row
        if ((x>=dayD[0]) && (x<=dayD[0]+29))  {                       //press day down
           hlightDownButton(dayD[0], dayD[1]);
           tmpRTC.tDay--;
           if (tmpRTC.tDay<=0) {tmpRTC.tDay = 31; }
        }
        if ((x>=monD[0]) && (x<=monD[0]+29))  {                       //press month down
           hlightDownButton(monD[0], monD[1]);
           tmpRTC.tMonth--;
           if (tmpRTC.tMonth<=0) {tmpRTC.tMonth = 12; }
        }   
        if ((x>=yeaD[0]) && (x<=yeaD[0]+29)) {                       //press year down
           hlightDownButton(yeaD[0], yeaD[1]);
           if (tmpRTC.tYear>2000)
             tmpRTC.tYear--;
        }
      }
      clockScreen(false);   
      }     
      break;
      
      case 3:                        //----------------------------------test LEDs screen--------------------------
        if ((x>=prOK[0]) && (x<=prOK[2]) && (y>=prOK[1]) && (y<=prOK[3])) {     //press reset
          hlightButton("RESET", prOK[0], prOK[1], prOK[2], prOK[3]);
	  LEDtestFlag = 0;
	  min_cnt=0;
	  for (int i=0; i<numberOfCh; i++)  { 
             prevLEDch_out[i] = 255; 
             LEDch_out[i] = 0;
	  }
          testScreen(true);
	}
	
	else if ((x>=stsT[0]) && (x<=stsT[2]) && (y>=stsT[1]) && (y<=stsT[3]))     //press start/stop test
          { 
           if (LEDtestFlag==1) {                                                      //stop
             hlightButton("RESUME TEST", stsT[0], stsT[1], stsT[2], stsT[3]);
             LEDtestFlag = 2;
             testScreen();
             }
           else {                                                                  //start
             hlightButton("STOP TEST", stsT[0], stsT[1], stsT[2], stsT[3]);
             if (LEDtestFlag == 0) {  
               min_cnt=0;        
               for (int i=0; i<numberOfCh; i++)  { 
                 prevLEDch_out[i] = 255; 
                 LEDch_out[i] = 0; 
               }
             }
             LEDtestFlag = 1;
             testScreen();
           }
          } else
            {
             if ((x>=tenM[0]) && (x<=tenM[2]) && (y>=tenM[1]) && (y<=tenM[3]))      //press -10s
               {
                min_cnt -= 10;
                if (min_cnt<0) {
                   min_cnt= 0; }
                delay(50);
                }
             if ((x>=tenP[0]) && (x<=tenP[2]) && (y>=tenP[1]) && (y<=tenP[2]))     //press +10s
               {
                min_cnt += 10;
                delay(50);
                }
              }
      break;
      
     case 4:             //---------------------------------------Temperature Settings ----------------------
       if ((x>=prOK[0]) && (x<=prOK[2]) && (y>=prOK[1]) && (y<=prOK[3]))       //press save
        {
        hlightButton("SAVE", prOK[0], prOK[1], prOK[2], prOK[3]);
        tempVal = tmpTempVal;
        dispScreen=0;
        SaveToEEPROM(6);
        clearScreen();
        mainScreen(true);
        } 
        else {
          if ((y>=temP[1]) && (y<=temP[1]+23)) {                        //first row 
            if ((x>=temP[0]) && (x<=temP[0]+29)) {                     //press temp plus
               hlightUpButton(temP[0], temP[1]);
               if (tmpTempVal.tempSet<50) 
                 tmpTempVal.tempSet += 0.1;
            }
            if ((x>=offP[0]) && (x<=offP[0]+29)) {                     //press +- plus
               hlightUpButton(offP[0], offP[1]);   
               if (tmpTempVal.tempOff < 10) 
                 tmpTempVal.tempOff += 0.1;
            }
          }
          else if ((y>=temM[1]) && (y<=temM[1]+23)) {                        //second row
            if ((x>=temM[0]) && (x<=temM[0]+29)) {                     //press temp minus
               hlightDownButton(temM[0], temM[1]);
               if (tmpTempVal.tempSet>10)
                 tmpTempVal.tempSet -= 0.1;
            }  
            if ((x>=offM[0]) && (x<=offM[0]+29)) {                      //press +- minus
               hlightDownButton(offM[0], offM[1]);
               if (tmpTempVal.tempOff > 0) 
                 tmpTempVal.tempOff -= 0.1;
            }          
          }
/*          else if ((y>=hemP[1]) && (y<=hemP[1]+23)) {                        //third row 
            if ((x>=hemP[0]) && (x<=hemP[0]+29)) {                     //press LEDs min temp plus
               hlightUpButton(hemP[0], hemP[1]);
               if ((tmpTempVal.tempLEDmin<90) && (tmpTempVal.tempLEDmin < tmpTempVal.tempLEDmax))
                 tmpTempVal.tempLEDmin += 1;
             }
             if ((x>=hetP[0]) && (x<=hetP[0]+29)) {                     //press LEDs max temp plus
               hlightUpButton(hetP[0], hetP[1]); 
               if (tmpTempVal.tempLEDmax<90) 
                 tmpTempVal.tempLEDmax += 1;
             }
          } 
           else if ((y>=hemM[1]) && (y<=hemM[1]+23)) {                     //fourth row
              if ((x>=hemM[0]) && (x<=hemM[0]+29)) {                     //press LEDs min temp minus
                hlightDownButton(hemM[0], hemM[1]); 
                if (tmpTempVal.tempLEDmin>10) 
                  tmpTempVal.tempLEDmin -= 1;
              }
              if ((x>=hetM[0]) && (x<=hetM[0]+29)) {                     //press LEDs max temp minus
                hlightDownButton(hetM[0], hetM[1]);
                if ((tmpTempVal.tempLEDmax>10) && (tmpTempVal.tempLEDmax > tmpTempVal.tempLEDmin))
                  tmpTempVal.tempLEDmax -= 1;
              }
           }
           else if ((y>=hecP[1]) && (y<=hecP[1]+23)) {                          //fifth row, LEDs cut off temp.
              if ((x>=hecP[0]) && (x<=hecP[0]+29)) {                       //press LEDs cut off plus
                hlightUpButton(hecP[0], hecP[1]);
                tmpTempVal.tempLEDcut += 1;
                if (tmpTempVal.tempLEDcut<20) {tmpTempVal.tempLEDcut = 20; }
                else if (tmpTempVal.tempLEDcut>99) {tmpTempVal.tempLEDcut = 99; }
              }
           }
           else if ((y>=hecM[1]) && (y<=hecM[1]+23)) {                          //sixth row,
              if ((x>=hecM[0]) && (x<=hecM[0]+29)) {                       //press LEDs cut off minus
                hlightDownButton(hecM[0], hecM[1]);
                tmpTempVal.tempLEDcut -= 1;
                if (tmpTempVal.tempLEDcut<20) {tmpTempVal.tempLEDcut = 0; }
                }
            }
*/        tempScreen();
          }
         break;
     
    case 5:                      //--------------------------------------menu select LED channel---------------------------
       if ((x>=ch1L[0]) && (x<=ch1L[2])) {              //first column
         if ((y>=ch1L[1]) && (y<=ch1L[3]))                      //press channel 1
           {
            hlightButton(ch1Descr, ch1L[0], ch1L[1], ch1L[2], ch1L[3]);
            selectChannel = 1;
            dispScreen=6;
            clearScreen();
            ledSetScreen();
           }
         if ((y>=ch3L[1]) && (y<=ch3L[3]) && (numberOfCh>2))                      //press channel 3
           {
            hlightButton(ch3Descr, ch3L[0], ch3L[1], ch3L[2], ch3L[3]);
            selectChannel = 3;
            dispScreen=6;
            clearScreen();
            ledSetScreen();
           }
/*         if ((y>=ch5L[1]) && (y<=ch5L[3]) && (numberOfCh>4))                      //press channel 5
           {
            hlightButton(ch5Descr, ch5L[0], ch5L[1], ch5L[2], ch5L[3]);
            selectChannel = 5;
            dispScreen=6;
            clearScreen();
            ledSetScreen();
           }  */
         }
       if ((x>=ch2L[0]) && (x<=ch2L[2]))               //second column
         {
         if ((y>=ch2L[1]) && (y<=ch2L[3]))                      //press channel 2
           {
            hlightButton(ch2Descr, ch2L[0], ch2L[1], ch2L[2], ch2L[3]);
            selectChannel = 2;
            dispScreen=6;
            clearScreen();
            ledSetScreen();
           }
         if ((y>=ch4L[1]) && (y<=ch4L[3]) && (numberOfCh>3))                      //press channel 4
           {
            hlightButton(ch4Descr, ch4L[0], ch4L[1], ch4L[2], ch4L[3]);
            selectChannel = 4;
            dispScreen=6;
            clearScreen();
            ledSetScreen();
           }
         if ((y>=mooL[1]) && (y<=mooL[3]))                      //press moonlights
           {
            hlightButton("MOONLIGHTS", mooL[0], mooL[1], mooL[2], mooL[3]);
            dispScreen=9;
            clearScreen();
            moonScreen(true);
           }
         }
       break; 
         
     case 6:                   // --------------------------------show leds values table-----------------------
         if ((x>=prOK[0]) && (x<=prOK[2]) && (y>=prOK[1]) && (y<=prOK[3]))       //press change
            {
             hlightButton("CHANGE", prOK[0], prOK[1], prOK[2], prOK[3]);
             dispScreen=7;
             clearScreen(); 
             ledChangeScreen();
            } 
         else if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3]))  //press BACK
           {
            hlightButton("BACK", canC[0], canC[1], canC[2], canC[3]);
            dispScreen=5;
            clearScreen();
            LEDmenuScreen(); 
           }
      break;
      
      case 7:             //----------------------------------change leds values-----------------------------
        if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3]))  //press CANCEL
           {
            hlightButton("CANCEL", canC[0], canC[1], canC[2], canC[3]);
            dispScreen=6;
            clearScreen();
            ledSetScreen();
           }
        if ((x>=prOK[0]) && (x<=prOK[2]) && (y>=prOK[1]) && (y<=prOK[3]))       //press SAVE
          {
           hlightButton("SAVE", prOK[0], prOK[1], prOK[2], prOK[3]);
           
           switch (selectChannel) {
           case 1:
             for (int i=0; i<96; i++)
               ch1led[i]=tled[i];
             SaveToEEPROM(1);
           break;
           case 2:
             for (int i=0; i<96; i++)
               ch2led[i]=tled[i];
             SaveToEEPROM(2);
           break;
           case 3:
             for (int i=0; i<96; i++)
               ch3led[i]=tled[i];
             SaveToEEPROM(3);
           break;
           case 4:
             for (int i=0; i<96; i++)
               ch4led[i]=tled[i];
             SaveToEEPROM(4);
           break;
           case 5:
             for (int i=0; i<96; i++)
               ch5led[i]=tled[i];
             SaveToEEPROM(5);
           break;
           }
           dispScreen=6;
           clearScreen();
           ledSetScreen();
          }
          
          else if ((y>=15) && (y<=43))                                    //top row with times was touched
            {
             if ((x>=4) && (x<=316))
               {
                int oldLCT = LedChangTime;
                LedChangTime = map(x, 3, 320, 0, 12);                
                
                if (oldLCT != LedChangTime)                        //highlight touched time
                  {
                   myGLCD.setColor(0, 0, 0);
                   myGLCD.fillRect((oldLCT*26)+5, 20, (oldLCT*26)+29, 43);
                   changeFont(SMALL, 255, 255, 0, 0, 0, 0);
                   myGLCD.printNumI((oldLCT*2), (oldLCT*26)+19-(intlen(oldLCT*2)*4), 20);
                   myGLCD.printNumI(((oldLCT*2)+1), (oldLCT*26)+19-(intlen(oldLCT*2)*4), 31);
                   myGLCD.setColor(255, 0, 0);
                   myGLCD.fillRect((LedChangTime*26)+5, 20, (LedChangTime*26)+29, 43);
                   changeFont(SMALL, 255, 255, 255, 255, 0, 0);
                   myGLCD.printNumI((LedChangTime*2), (LedChangTime*26)+19-(intlen(LedChangTime*2)*4), 20);
                   myGLCD.printNumI(((LedChangTime*2)+1), (LedChangTime*26)+19-(intlen(LedChangTime*2)*4), 31);
                   
                   for (int i=0; i<8; i++)                          //print led values for highlighted time
                     {
                      int k=(LedChangTime*8)+i;
                      printVBar( tled[k], (i*38)+10, 44, rgbChann[selectChannel-1]);
                     }
                  }
               }
            } 
            else if ((y>=44) && (y<=65))       //plus buttons were touched
              {
               for (int i=0; i<8; i++) {                
                if ((x>=(i*38)+10) && (x<=(i*38)+29)) {
                  int k= (LedChangTime*8)+i;
                  if (tled[k]<100) 
                     tled[k]++;
                  printVBar( tled[k], (i*38)+10, 44, rgbChann[selectChannel-1]);
                 }
                }
              }
              else if ((y>=67) && (y<=166)) {                  //the bar has been touched
                for (int i=0; i<8; i++) {                
                  if ((x>=(i*38)+10) && (x<=(i*38)+29)) {
                    int k= (LedChangTime*8)+i;
                    tled[k] = 168-y;
                    printVBar( tled[k], (i*38)+10, 44, rgbChann[selectChannel-1]);
                  }
                }
              }   
              else if ((y>=168) && (y<=190))     //minus buttons were touched
                {
                 for (int i=0; i<8; i++) {                
                  if ((x>=(i*38)+10) && (x<=(i*38)+29)) {
                     int k= (LedChangTime*8)+i;
                     if (tled[k]>0) 
                       tled[k]--; 
                     printVBar( tled[k], (i*38)+10, 44, rgbChann[selectChannel-1]);
                    }
                  }  
                }
      break;
      
/*      case 8:                   // --------------------------------ATO settings-----------------------
        if ((x>=prOK[0]) && (x<=prOK[2]) && (y>=prOK[1]) && (y<=prOK[3])) {      //press ok
           hlightButton("SAVE", prOK[0], prOK[1], prOK[2], prOK[3]);
           ATOflag = tmpATOflag;
           ATOmax = tmpATOmax;
           SaveToEEPROM(7);
           dispScreen=0;
           clearScreen(); 
           mainScreen(true);
           }
        else {
          if ((x>=atoO[0]) && (x<=(atoO[0]+80)) && (y>=atoO[1]) && (y<=(atoO[1]+25))) {   // on/off switch
            if (tmpATOflag == 0)  
               tmpATOflag = 1;
            else if (tmpATOflag >=1) 
               tmpATOflag = 0; 
          }
          if ((y>=atoP[1]) && (y<=atoP[1]+23)) {
            if ((x>=atoP[0]) && (x<=atoP[0]+29))  {                      //press plus
               hlightUpButton(atoP[0], atoP[1]);
               tmpATOmax += 1; 
            }
          }        
          if ((y>=atoM[1]) && (y<=atoM[1]+23)) {                          //ato time change buttons
              if ((x>=atoM[0]) && (x<=atoM[0]+29)) {                       //press minus
                 hlightDownButton(atoM[0], atoM[1]);
                 if (tmpATOmax>1) 
                    tmpATOmax -= 1;
                }
          }
          ATOScreen();
        }
      break;
*/      
      case 9:    //----------------------------------MoonLight Settings----------------------
        if ((x>=prOK[0]) && (x<=prOK[2]) && (y>=prOK[1]) && (y<=prOK[3]))       //press ok
          {
           hlightButton("SAVE", prOK[0], prOK[1], prOK[2], prOK[3]);
           moonSett = tmpMoonSett;
           SaveToEEPROM(8);
           moon_init();
           moonlight_output();
           dispScreen=5;
           clearScreen();
           LEDmenuScreen();
          }
        else if ((x>=canC[0]) && (x<=canC[2]) && (y>=canC[1]) && (y<=canC[3]))  //press BACK
           {
            hlightButton("BACK", canC[0], canC[1], canC[2], canC[3]);
            dispScreen=5;
            clearScreen();
            LEDmenuScreen(); 
           }
           
        else if ((x>=morO[0]) && (x<=(morO[0]+80)) && (y>=morO[1]) && (y<=(morO[1]+25)))    // on/off switch
          {
           tmpMoonSett.timesReal = !tmpMoonSett.timesReal;
           moonScreen();
          }
        else {
        if ((y>=morP[1]) && (y<=morP[1]+23))                           //moonlights time change buttons
         {
          if ((x>=morP[0]) && (x<=morP[0]+29)) {                 //press moonrise plus
             hlightUpButton(morP[0], morP[1]);
             if (tmpMoonSett.moonrise < 23)
               tmpMoonSett.moonrise += 1;
          }   
          else if ((x>=mosP[0]) && (x<=mosP[0]+29)) {                 //press moonset plus
             hlightUpButton(mosP[0], mosP[1]);
             if ((tmpMoonSett.moonset < 23) && (tmpMoonSett.moonset < tmpMoonSett.moonrise))
               tmpMoonSett.moonset += 1;
           }
         } 
         else if ((y>=morM[1]) && (y<=morM[1]+23)) {
           if ((x>=morM[0]) && (x<=morM[0]+29)) {                //press moonrise minus
             hlightDownButton(morM[0], morM[1]);
             if ((tmpMoonSett.moonrise > 0) && (tmpMoonSett.moonrise > tmpMoonSett.moonset))
               tmpMoonSett.moonrise -= 1;
           } 
           else if ((x>=mosM[0]) && (x<=mosM[0]+29)) {                //press moonset minus
             hlightDownButton(mosM[0], mosM[1]);
             if (tmpMoonSett.moonset > 0)
               tmpMoonSett.moonset -= 1;
           } 
         } 
         else if ((y>=nmlP[1]) && (y<=nmlP[1]+23)) {                          //moonlight lights values
           if ((x>=nmlP[0]) && (x<=nmlP[0]+29)) {                        //new moon light plus
             hlightUpButton(nmlP[0], nmlP[1]);
             if ((tmpMoonSett.newMoonLight < 100) && (tmpMoonSett.newMoonLight < tmpMoonSett.fullMoonLight))
               tmpMoonSett.newMoonLight += 1;
           }
           else if ((x>=fmlP[0]) && (x<=fmlP[0]+29)) {                      //full moon light plus
             hlightUpButton(fmlP[0], fmlP[1]);
             if (tmpMoonSett.fullMoonLight < 100)
               tmpMoonSett.fullMoonLight += 1;
           }
         } 
         else if ((y>=nmlM[1]) && (y<=nmlM[1]+23)) { 
           if ((x>=nmlM[0]) && (x<=nmlM[0]+29)) {                //new moon light minus
             hlightDownButton(nmlM[0], nmlM[1]);
             if (tmpMoonSett.newMoonLight > 0)
               tmpMoonSett.newMoonLight -= 1;
           } 
           else if ((x>=fmlM[0]) && (x<=fmlM[0]+29)) {          //full moon light minus
             hlightDownButton(fmlM[0], fmlM[1]);
             if ((tmpMoonSett.fullMoonLight > 0) && (tmpMoonSett.fullMoonLight > tmpMoonSett.newMoonLight))
               tmpMoonSett.fullMoonLight -= 1;
           }
         }     
       moonScreen();    
       }
       break;
       
       case 10:    //---------------------------------- General Settings ------------------------
         if ((x>=prOK[0]) && (x<=prOK[2]) && (y>=prOK[1]) && (y<=prOK[3]))       //press ok
          {
           hlightButton("SAVE", prOK[0], prOK[1], prOK[2], prOK[3]);
           LCDbright = tmpLCDbright;
           byte bout = map(LCDbright, 0, 100, 0, 255);
           analogWrite(LCDbrightPin, bout);
	   SaveToEEPROM(9);
           dispScreen=0;
           clearScreen(); 
           mainScreen(true);
          }
         else if ((x>=gseB[0]) && (x<=gseB[0]+29) && (y>=gseB[1]) && (y<=gseB[1]+145)) {
           if ((y>=gseB[1]) && (y<=gseB[1]+21)) {       //plus
             hlightUpButton(gseB[0], gseB[1]);
             if (tmpLCDbright <100)
               tmpLCDbright++;
           }
           else if ((y>=gseB[1]+23) && (y<=gseB[1]+122))
             tmpLCDbright = gseB[1] + 122 - y;
           else if ((y>=gseB[1]+124) && (y<=gseB[1]+145)) {      //minus
             hlightDownButton(gseB[0], gseB[1]+124);
             if (tmpLCDbright>0)
               tmpLCDbright--;
           }
          byte bout = map(tmpLCDbright, 0, 100, 0, 255);
          analogWrite(LCDbrightPin, bout);
          genSetScreen();
         }
         else if ((x>=gseS[0]) && (x<=gseS[2])) {
           if ((y>=gseS[1]) && (y<=gseS[3])) {             //save
             hlightButton("SAVE", gseS[0], gseS[1], gseS[2], gseS[3]);
             for (int i=1; i<10; i++)
               SaveToEEPROM(i);
           }
           if ((y>=gseL[1]) && (y<=gseL[3])) {             //load
             hlightButton("LOAD", gseL[0], gseL[1], gseL[2], gseL[3]);
             ReadFromEEPROM();
           }
         }
       break;
       
       case 11:    //---------------------------------- Color Mixer ------------------------
         int px, out;
         
         for (int i=0; i<numberOfCh; i++) { 
           px = i*(270/numberOfCh) + ((318/numberOfCh)/2+2);
           if ((x>=px) && (x<=px+29)) {
             if ((y>=22) && (y<=43))  {                            //plus buttons were touched
               if (LEDch_out[i]<100) 
                 LEDch_out[i]++;
             } 
             else if ((y>=45) && (y<=144)) {                        //the bar has been touched
               LEDch_out[i] = 144-y;
             }
             else if ((y>=146) && (y<=167)) {                        //minus buttons were touched
               if (LEDch_out[i]>0) 
                 LEDch_out[i]--; 
             }
           
             printVBar( LEDch_out[i], px, 22, rgbChann[i]);
             if (BUCKPUCK) 
               out = LEDch_out[i];
             else 
               out = 100 - LEDch_out[i];
             out = map(out, 0, 100, 0, 255);    
             analogWrite(LEDpin[i], out);
           }
         }
       break;
       
    } 
  } 
  delay(100);
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//                                             SETUP
/////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
  initSettings();
  
  TCCR3A = B00100011;        // Fast PWM on timer 3
  TCCR3B = B11001;           //no prescalering
  OCR3A = 639;               //count to 639 (16MHz/(640-1)=25 kHz)

  pinMode(ledPinCh1, OUTPUT);
  digitalWrite(ledPinCh1, LOW);
  pinMode(ledPinCh2, OUTPUT);
  digitalWrite(ledPinCh2, LOW);
  pinMode(ledPinCh3, OUTPUT);
  digitalWrite(ledPinCh3, LOW);
  pinMode(ledPinCh4, OUTPUT);
  digitalWrite(ledPinCh4, LOW);
//  pinMode(ledPinCh5, OUTPUT);
//  digitalWrite(ledPinCh5, LOW);
  pinMode(moonlightPin, OUTPUT);
  digitalWrite(moonlightPin, LOW);
/*  pinMode(AlarmBuzzPin, OUTPUT);
  pinMode(fan1PWMpin, OUTPUT);
  //digitalWrite(fan1PWMpin, LOW);            
  pinMode(fan2PWMpin, OUTPUT);
  //digitalWrite(fan2PWMpin, LOW);            
  pinMode(fan1TranzPin, OUTPUT); */
  pinMode(LCDbrightPin, OUTPUT);
  analogWrite(LCDbrightPin, LCDbright);
/*  
  pinMode(A1, OUTPUT);
  digitalWrite(A1, LOW);
  pinMode(19, INPUT);
  digitalWrite(19, HIGH);
  attachInterrupt(4, pumpOn, CHANGE);
*/
  myGLCD.InitLCD(LANDSCAPE);
  myGLCD.clrScr();
  
  myTouch.InitTouch(LANDSCAPE);
  myTouch.setPrecision(PREC_MEDIUM);
  
/*  sensorW.begin();           //start up temperature library
  sensorH.begin();           //start up temperature library
  sensorW.getAddress(waterSensor, 0);   //get addresses of temperature sensors
  sensorH.getAddress(ledSensor, 0);    */
  
  pinMode(sensorW, INPUT);			//set analog pin as input for water temp
  digitalWrite(sensorW, LOW);  		//set pull-up on analog pin 
  pinMode(sensorWPower, OUTPUT);	//initiate water temp sensor power pin
  digitalWrite(sensorWPower, HIGH);	//set pin high, this will turn on water temp sensor
  
  checkTempC();
//  checkTempH();
  
  myGLCD.setColor(64, 64, 64);
  myGLCD.drawRect(0, 0, 319, 239);
  myGLCD.fillRect(0, 226, 319, 239);
  changeFont(SMALL, 255, 255, 0, 64, 64, 64);
  myGLCD.print("  Stilo v.3.0", LEFT, 227);
  myGLCD.setColor(255, 128, 128);
  myGLCD.print("Aquarium Controller", RIGHT, 227);
    
  RTC.getTime();
  //min_cnt= (RTC.hour*60)+RTC.minute;
  if (readEEonStart) {
    ReadFromEEPROM(); }
  LED_levels_output();  
  
  moon_init();
  moonlight_output();
  
  mainScreen(true);
  
  //delay (3000);
  //Serial.println(tmpTempVal.tempSet);
  //Serial.println(tmpTempVal.tempOff);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//                                    MAIN LOOP
/////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
 {
  unsigned long currentMillis = millis();
  
  if (myTouch.dataAvailable())  {
    processMyTouch();
   // prevMillis600 = currentMillis;
  }
      
/*  else if (currentMillis - prevMillis1 > 1000)
    {
      prevMillis1 = currentMillis;
      if ((digitalRead(A1) == HIGH) && (ATOflag == 1)) {
        ATOflag = 2;
        ATOstart = currentMillis;
      }
      else if ((digitalRead(A1) == HIGH) && (ATOflag == 2)) {
        if ((currentMillis - ATOstart) > (ATOmax*1000)) {        //ato switch on for too long
          ATOflag = 3;                                           //mark ato as failed
          digitalWrite (A1, LOW);
        }
      } 
    }
*/  
  else if (currentMillis - prevMillis5 > 5000)    //check time, temp and LED levels every 5s
    {      
      prevMillis5 = currentMillis;  
      
      RTC.getTime();
      
      if (tempCorHflag == 0) {
        checkTempC();
        tempCorHflag = !tempCorHflag;
      }
      
      if (dispScreen!=11) 
		LED_levels_output();
		moonlight_output();
      }
      if (dispScreen == 0)
		mainScreen(); 
    }    
  else if (currentMillis - prevMillis60 > 60000)    //check moonlight every 1 min
    {
      prevMillis60 = currentMillis;
      
      if (moonSett.timesReal == 0) {
        if (((moonSett.moonrise == RTC.hour) || (moonSett.moonset == RTC.hour)) && (RTC.minute == 0)) {
          moon_init();
          moonlight_output();
          if (dispScreen==0) {
           clearScreen(); 
           mainScreen(true);
          }
        }
      } 
      else if (((Moon.riseH == RTC.hour) && (Moon.riseM == RTC.minute)) || ((Moon.setH == RTC.hour) && (Moon.setM == RTC.minute))) {
        moon_init();
        moonlight_output();
        if (dispScreen==0) {
          clearScreen(); 
          mainScreen(true);
        }
      }
    }
  else if (currentMillis - prevMillis600 > 600000)    //back to main screen after 10 min
  {
    if (dispScreen!=0) {
      prevMillis600 = currentMillis;
      LEDtestFlag = 0;
      byte bout = map(LCDbright, 0, 100, 0, 255);
      analogWrite(LCDbrightPin, bout);
      dispScreen=0;
      clearScreen();
      mainScreen(true);
    }
  }
 } //-------------------end of main loop
