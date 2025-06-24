void setupRTC(){
  Wire.begin();

  rtc.begin();

    // if (!rtc.begin()) {
    //     Serial.println("Couldn't find RTC");
    //     while (1);
    // }

    if (rtc.lostPower()) {
        Serial.println("RTC lost power, setting default time...");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set to compile time
    }
}