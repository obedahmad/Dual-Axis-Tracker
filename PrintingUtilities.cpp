#include <TimeLib.h>
#include <SolarTracker.h>

byte hourRun, minRun, secondRun, dayRun, monthRun, yearRun; //Segmented tmElements_t for printing Local Time 
double elevation, azimuth, zenith;

void printTime(time_t t){
tmElements_t someTime;
breakTime(t, someTime);

hourRun=someTime.Hour;
minRun=someTime.Minute;
secondRun=someTime.Second;

dayRun=someTime.Day;
monthRun=someTime.Month;
yearRun = someTime.Year;

Serial.print(hourRun);
Serial.print(F(":"));
Serial.print(minRun);
Serial.print(F(":"));
Serial.print(secondRun);
Serial.print("  ");
Serial.print(dayRun);
Serial.print(F("|"));
Serial.print(monthRun);
Serial.print(F("|"));
Serial.print(yearRun);
}

void printSolarPosition(SolarTrackerStruct pos, int numDigits) {
elevation = pos.elevation;
azimuth = pos.azimuth;
zenith = abs(elevation-90);

Serial.print(F("az: "));Serial.print(azimuth,numDigits);Serial.print(F(" deg"));
Serial.print("  "); 
Serial.print(F("el: "));Serial.print(elevation,numDigits);
Serial.print("  "); 
Serial.print(F("ze: "));Serial.print(zenith,numDigits);Serial.print(F(" deg"));
Serial.println(F(" deg\t"));
}