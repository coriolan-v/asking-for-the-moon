void setupRTC(){
  Wire.begin();
#if defined(TWBR)
  TWBR = 12; // Increase I2C speed (400kHz on 16MHz MCU)
#endif
  rtc.begin();
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting default time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}
