#include "bluefruit.h"
#include <RTClib.h>
#include <Wire.h>
RTC_DS3231 rtc;

void setup() 
{
  Serial.begin(9600);

  setupRTC();

  setupSteppers();

  homeSteppers();  
}

void loop() 
{
  readMoonData();

  runSteppersWithStatus();
}
