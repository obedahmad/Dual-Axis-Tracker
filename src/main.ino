//REQUIRED INCLUDES FOR HARDWARE/COMPUTING
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <SolarTracker.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <DS3232RTC.h>
#include <TimeLib.h>
#include "MotorRoutines.h"
#include "PrintingUtilities.h"
//BLYNK IoT Definitions
#define BLYNK_TEMPLATE_ID "TMPLCzpL9-5g"
#define BLYNK_DEVICE_NAME "Tracker"
#define BLYNK_AUTH_TOKEN "eH-s81BRkJ-P6t8ywjySKwmp9xJhg61i"
#define BLYNK_PRINT Serial
void myTimerEvent(){
  //You can send any value at any time
  //Please don't send more than 10 values per second
  Blynk.virtualWrite(V0,0);
  Blynk.virtualWrite(V3,String(azimuth));
  Blynk.virtualWrite(V4,String(elevation));
}

DS3232RTC RTC;
LiquidCrystal_I2C lcd(0x27,20,4);
BlynkTimer timer;
char ssid[] = "TP-LINK_DCD2A4";
char pass[] = "40886d74";

#define RXD1 26
#define TXD1 25

//- - - - - - - - - MECHANICAL CONTRAINTS - - - - - - - - -
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - -
#define UP_LIMIT_DEG   60   //Elevation Up limit of tracker
#define DOWN_LIMIT_DEG 0    //Elevation Down limit of tracker
#define EAST_LIMIT_DEG 120  //Azimuth East limit of tracker
#define WEST_LIMIT_DEG 240  //Azimuth West limit of tracker
#define SOUTH_DEG      180  //Reference Degrees of South

//- - - - - - - - - - - MOTOR SETUP - - - - - - - - - - - -
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//- - - - - - - - - - PIN DESIGNATIONS  - - - - - - - - - -
//Potentiometer Pin Designation
#define elev_pot 35       // Analog input pin for elevation potentiometer
#define azim_pot 34       // Analog input pin for azimuth potentiometer
//Motor Driver 1 Pin Designation
const int M1_LPwm = 19;   // I/O channel setup ESP32 pin 
const int M1_RPwm = 18;   // I/O channel setup ESP32 pin
const int En_M1    = 5;   // I/O pin for BTS Driver Enable
//Motor Driver 2 Pin Designation
const int M2_LPwm = 2;    // I/O channel setup ESP32 pin 
const int M2_RPwm = 4;    // I/O chennel setup ESP32 pin
const int En_M2    =15;   // I/O pin for BTS Driver Enable
//Channel Variables
const int  freq = 1000;   // PWM Frequency
const int  res = 8;       // PWM Resolution
//- - - - - - - - - - VALUE ASSIGNMENTS  - - - - - - - - - -
//Potentiometer Value Assignment
int pot1value = 2047;     // Potentiometer Analog Raw Value Scale
int dirM1 = 2;            // Motor control: dirM1 {1=leftTurn 2=stop 3=rightTurn}
int Speed1 = 255 ;        // Speed Control (@maxSpeed)
int pot2value = 2047;     // Potentiometer Analog Raw Value Scale
int dirM2 = 2;            // Motor control: dirM1 {1=leftTurn 2=stop 3=rightTurn}
int Speed2 = 255 ;        // Speed Control (@maxSpeed)
const int  Channel_15 = 15;   // PWM channel 0, for BTS pin M1_LPwm 
const int  Channel_14 = 14;   // PWM channel 1, for BTS pin M1_RPwm
const int  Channel_13 = 13;   // PWM channel 0, for BTS pin M2_LPwm
const int  Channel_12 = 12;   // PWM channel 1, for BTS pin M2_RPwm
//Control Flow Flags' Assignment 
int elev_pot_up_limit=450;    // Potentiometer Reading at Elevation Up limit of tracker
int elev_pot_down_limit=0;    // Potentiometer Reading at Elevation Down limit of tracker
int azim_pot_east_limit=0;    // Potentiometer Reading at Azimuth East limit of tracker
int azim_pot_west_limit=600;  // Potentiometer Reading at Azimuth West limit of tracker
int num_pot_readings=10;
int motion_flag=1;        //General Motion Flag for movement 8=UP, 4=DOWN, 2=EAST, 1=WEST, 0=STOP
int autoMoveElevFlag;     //Motion Flag for Azimuth Movement
int autoMoveAzimFlag;     //Motion Flag for Elevation Movement
int autoMoveElevDeg;      //Motion Flag for Elevation Movement
int autoMoveAzimDeg;      //Motion Flag for Azimuth Movement

//Control Variable Declaration
double elevation;
double azimuth;
const uint8_t digits = 3;
int ledPin = 02;
char daysOfTheWeek[7][12]={"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
byte hourRun, mintRun, secondRun,dateRun,monthRun; //Segmented tmElements_t for printing Local Time 
String  lat_str,lng_str;

float latitude=34.0691;
float longitude=72.6441;
SolarTracker SolarObject(34.0691, 72.6441); //SolarTracker object initialized with coordinates for GIKI
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

//- - - - - - - - - - SETUP FUNCTIONS - - - - - - - - - - -
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void setup(){

Serial.begin(115200);
SerialGPS.begin(9600, SERIAL_8N1,RXD1,TXD1);//initRTC();

///Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
// SolarTracker::setTimeProvider(RTC.get);
//setSyncProvider(myRTC.get);   // the function to get the time from the RTC
RTC.begin();
if(timeStatus() != timeSet) Serial.println("Unable to sync with the RTC");
else Serial.println("RTC has set the system time");
setTime(8,30,0,12,4,2023); //Intial Setpoint for timevalue
time_t t = now();
RTC.set(now());

lcd.init();
lcd.backlight();
lcd.setCursor(0,0);
lcd.print("DAST SYS");

pinMode(ledPin,OUTPUT);
digitalWrite(ledPin,HIGH); //OFF

ledcSetup(Channel_15, freq, res); // setup PWM channel for BST L_PWM
ledcSetup(Channel_14, freq, res); // setup PWM channel for BST R_PWM
ledcAttachPin( M1_LPwm , Channel_15); // Attach BST L_PWM
ledcAttachPin( M1_RPwm , Channel_14); // Attach BST R_PWM

ledcSetup(Channel_13, freq,res); // setup PWM channel for BST L_PWM
ledcSetup(Channel_12, freq,res); // setup PWM channel for BST R_PWM
ledcAttachPin( M2_LPwm , Channel_13); // Attach BST L_PWM
ledcAttachPin( M2_RPwm , Channel_12); // Attach BST R_PWM

Serial.println("Program Start");
timer.setInterval(1000L, myTimerEvent); //BLYNK App Intiation
}

//- - - - - - - - - - LOOPS FUNCTIONS - - - - - - - - - - -
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void loop(){
//Blynk.run();
//timer.run(); // Initiates BlynkTimer

while(SerialGPS.available() > 0){
  if(gps.encode(SerialGPS.read())){
  digitalWrite(ledPin,!digitalRead(ledPin));//ON
  if(gps.location.isValid()){
          latitude = gps.location.lat();
          lat_str = String(latitude , 6);
          longitude = gps.location.lng();
          lng_str = String(longitude , 6);
          Serial.print("GPS Lat = ");
          Serial.println(lat_str);
          Serial.print("GPS Lon = ");
          Serial.println(lng_str);
        } else
        {
          lat_str="34.0691";
          lng_str="72.6441"; 
        }
  }
}
Serial.println("update Location");

printTime(RTC.get());
Serial.println("");
Serial.print(F("Topi:\t"));
printSolarPosition(SolarObject.getSolarPosition(RTC.get()),digits);

lcd.setCursor(0,1);lcd.print("x:");lcd.print(lat_str);lcd.print(" y:");lcd.print(lng_str);
lcd.setCursor(0,2);lcd.print("ez:");lcd.print(azimuth);lcd.print(" el:");lcd.print(elevation);
lcd.setCursor(0,3);lcd.print("x:");lcd.print(read_azim_pot()); 
lcd.setCursor(10,3);lcd.print(" y:");lcd.print(read_elev_pot());

if(elevation<=0){
  autoMoveElevDeg=UP_LIMIT_DEG;//90
  autoMoveAzimDeg=SOUTH_DEG;//180
  Serial.println("BOUNDARY CONDITIONS (LINE 280)");
}

if(elevation>0){////30
  if(elevation<DOWN_LIMIT_DEG) 
    {
      autoMoveElevDeg=DOWN_LIMIT_DEG;
      Serial.println("ELEVATION DOWN LIMIT SET");
    }
   else
    {
      autoMoveElevDeg=elevation; 
      Serial.println("ELEVATION VALUE SET"); Serial.println(autoMoveElevDeg);
    }
  if(azimuth<EAST_LIMIT_DEG) {
    autoMoveAzimDeg=EAST_LIMIT_DEG;
    Serial.println("AZIMUTH EAST LIMIT SET");
    }
  else if(azimuth>WEST_LIMIT_DEG) {
    autoMoveAzimDeg=WEST_LIMIT_DEG;
    Serial.println("AZIMUTH WEST LIMIT SET");
    }
  else{
    autoMoveAzimDeg=azimuth; 
    Serial.println("AZIMUTH VALUE SET"); Serial.println(autoMoveAzimDeg);
    }
}

autoMoveElevFlag=0;
autoMoveAzimFlag=0;

if(elevation>=0)
{
  if(autoMoveElevFlag==0){
    automove_elev(autoMoveElevDeg, autoMoveElevFlag, Channel_15, Channel_14, Speed1);
    Serial.println("ELEVATION FLAG UP");
    }
  if(autoMoveAzimFlag==0){
    automove_azim(autoMoveAzimDeg, autoMoveAzimFlag, Channel_12, Channel_13, Speed2);
    Serial.println("AZIMUTH FLAG UP");
    }
}
Serial.print("Elev Flag: ");Serial.println(autoMoveElevFlag);
Serial.print("Azim Flag: ");Serial.println(autoMoveAzimFlag);

delay(1000);
}