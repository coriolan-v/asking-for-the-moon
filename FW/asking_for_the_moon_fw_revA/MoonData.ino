#include <avr/pgmspace.h>
#include <RTClib.h>
#include "moon_data.h"

// Auto-generated moon azimuth and altitude arrays for Arduino
#define AZIMUTH_OFFSET -279

const float ALTITUDE_DEG_AT_ZERO_STEPS = -90.0;   // arm pointing down
const float ALTITUDE_DEG_AT_FULL_STEPS = +90.0;   // arm pointing up


#define INTERVAL_MINUTES 5

// Hardcoded start time (when first moon data point occurs)
const int startYear = 2025;
const int startMonth = 6;
const int startDay = 24;
const int startHour = 0;
const int startMinute = 0;

// These must be declared in another file (like moon_data_arduino.h)
extern const float moonAzimuth[] PROGMEM;
extern const float moonAltitude[] PROGMEM;

extern const long stepper_altitude_fullRotation;
extern const long stepper_azimuth_fullRotation;

int lastIndexRead = -1;

void readMoonData() {
  static unsigned long lastCheckMillis = 0;
  static DateTime lastTime;
  static int lastSecond = -1;

  unsigned long nowMillis = millis();
  unsigned long checkInterval = 250; // check RTC only 4× per second (lightweight)

  if (nowMillis - lastCheckMillis < checkInterval)
    return;

  lastCheckMillis = nowMillis;

  DateTime now = rtc.now();

  // Avoid unnecessary processing if time hasn't changed
  if (now.second() == lastSecond) return;
  lastSecond = now.second();

  // Print timestamp every 1s while homing, every 20s otherwise
  static unsigned long lastPrint = 0;
  unsigned long printInterval = isHoming ? 1000 : 20000;

  if (nowMillis - lastPrint >= printInterval) {
    lastPrint = nowMillis;
    Serial.print("⏰ Time: ");
    Serial.println(now.timestamp());
  }

  if (isHoming) return;

  int index = getMoonDataIndex(now);

  // Only load moon data exactly on the 5-minute mark, once
  if (index != lastIndexRead && now.minute() % INTERVAL_MINUTES == 0 && now.second() == 0) {
    lastIndexRead = index;

    float az, alt;
    readMoonDataAtTime(now, az, alt);

    Serial.println("🌙 Reading moon data...");
    Serial.print("Azimuth: ");
    Serial.print(az);
    Serial.print(" | Altitude: ");
    Serial.println(alt);

    moveToMoonPosition(az, alt);
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

  Serial.print("DEBUG: Index = ");
  Serial.print(index);
  Serial.print(" | Azimuth read = ");
  Serial.print(azimuth);
  Serial.print(" | Altitude read = ");
  Serial.println(altitude);
}


long mapAzimuthToSteps(float azimuthDeg) {
  float adjustedAz = fmod((azimuthDeg + AZIMUTH_OFFSET + 360.0), 360.0);
  return lroundf(adjustedAz * (stepper_azimuth_fullRotation / 360.0f));
}

long mapAltitudeToSteps(float altitudeDeg) {
  float range = ALTITUDE_DEG_AT_FULL_STEPS - ALTITUDE_DEG_AT_ZERO_STEPS; // 180
  float shifted = ALTITUDE_DEG_AT_FULL_STEPS - altitudeDeg;  // Invert altitude mapping here
  return lroundf(shifted * (stepper_altitude_fullRotation / range));
}








//If instead 0° is horizontal and increases toward up, and you want 0° to map to 90° real altitude, then flip like:
// long mapAltitudeToSteps(float altitudeDeg) {
//   return (long)((90.0 - altitudeDeg) * (stepper_altitude_fullRotation / 360.0));
// }



