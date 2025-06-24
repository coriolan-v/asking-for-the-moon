#include <RTClib.h>
#include <Wire.h>
RTC_DS3231 rtc;
bool isHoming = false;

void setup() 
{
  Serial.begin(9600);

  setupRTC();

  setupSteppers();

  homeSteppers();  
}

void loop() {
  // Update moon position and check if we need to move the steppers
  readMoonData();

  // If currently homing, update homing logic instead of running steppers
  if (isHoming) {
    updateHoming();
  } else {
    runSteppersWithStatus();  // Perform normal lunar tracking
  }

  // Perform daily homing check at specific time
//  homeEveryDay();
}


