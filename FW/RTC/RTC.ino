#include <Wire.h>
#include <RTClib.h>

RTC_DS3231 rtc;

void setup() {
    Serial.begin(9600);
    Wire.begin();

    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        while (1);
    }

    if (rtc.lostPower()) {
        Serial.println("RTC lost power, setting default time...");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set to compile time
    }
}

void loop() {
    DateTime now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.println(now.second(), DEC);

    delay(1000); // Print every second
}