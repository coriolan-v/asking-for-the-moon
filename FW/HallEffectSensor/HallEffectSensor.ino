#include "bluefruit.h"

const int sensorPin = A0;   // analog input
bool homePos = false;

unsigned long lastReadTime = 0;
const unsigned long readInterval = 50; // ms

void setup() {
  Serial.begin(9600);
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastReadTime >= readInterval) {
    lastReadTime = currentTime;

    int sensorValue = analogRead(sensorPin);

    homePos = (sensorValue >= 700);

    if (homePos) {
      Serial.println("HOME");
    } else {
      Serial.println(sensorValue);
    }
  }
}
