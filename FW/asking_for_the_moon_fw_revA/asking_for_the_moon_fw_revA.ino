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

void loop() {

  readMoonData();

  runSteppersWithStatus();  // Main tracking movement

  updateHoming();           // Step-by-step homing
  
  homeEveryDay();           // Check daily homing trigger
}

