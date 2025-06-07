#include <avr/pgmspace.h>
#include <RTClib.h>
#include "moon_data.h"

// Auto-generated moon azimuth and altitude arrays for Arduino

#define INTERVAL_MINUTES 5

// Hardcoded start time (when first moon data point occurs)
const int startYear = 2025;
const int startMonth = 6;
const int startDay = 2;
const int startHour = 0;
const int startMinute = 0;

// These must be declared in another file (like moon_data_arduino.h)
extern const float moonAzimuth[] PROGMEM;
extern const float moonAltitude[] PROGMEM;

extern const long stepper_altitude_fullRotation;
extern const long stepper_azimuth_fullRotation;

int lastIndexRead = -1;


void readMoonData(){
  static unsigned long lastCheck = 0;
  unsigned long nowMillis = millis();

  if (nowMillis - lastCheck >= 1000) {
    lastCheck = nowMillis;

    DateTime now = rtc.now();
    int index = getMoonDataIndex(now);

    if (index != lastIndexRead && now.minute() % INTERVAL_MINUTES == 0 && now.second() == 0) {
      lastIndexRead = index;

      float az, alt;
      readMoonDataAtTime(now, az, alt);

      Serial.print("Azimuth: ");
      Serial.print(az);
      Serial.print(" | Altitude: ");
      Serial.println(alt);
    }
  }
}

int getMoonDataIndex(DateTime now) {
  DateTime start(startYear, startMonth, startDay, startHour, startMinute, 0);
  TimeSpan elapsed = now - start;
  int totalMinutes = elapsed.totalseconds() / 60;
  return totalMinutes / INTERVAL_MINUTES;
}

void readMoonDataAtTime(DateTime now, float &azimuth, float &altitude) {
  int index = getMoonDataIndex(now);
  if (index < 0) index = 0;

  azimuth = pgm_read_float_near(&moonAzimuth[index]);
  altitude = pgm_read_float_near(&moonAltitude[index]);
}

long mapAzimuthToSteps(float azimuthDeg) {
  return (long)(azimuthDeg * (stepper_azimuth_fullRotation / 360.0));
}

//If your physical mount maps altitude 0° to 0 steps, and increases as you go up:
long mapAltitudeToSteps(float altitudeDeg) {
  return (long)(altitudeDeg * (stepper_altitude_fullRotation / 360.0));
}

//If instead 0° is horizontal and increases toward up, and you want 0° to map to 90° real altitude, then flip like:
// long mapAltitudeToSteps(float altitudeDeg) {
//   return (long)((90.0 - altitudeDeg) * (stepper_altitude_fullRotation / 360.0));
// }



