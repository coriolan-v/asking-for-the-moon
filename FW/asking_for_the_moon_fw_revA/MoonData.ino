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

bool isHoming = false;


void readMoonData() {
  static unsigned long lastCheck = 0;
  static unsigned long lastTimePrint = 0;

  unsigned long nowMillis = millis();
  DateTime now = rtc.now();

  // Print every second if homing, every 20 seconds if NOT homing
  unsigned long interval = isHoming ? 1000 : 20000;

  if (nowMillis - lastTimePrint >= interval) {
    lastTimePrint = nowMillis;

    Serial.print("⏰ Time: ");
    Serial.println(now.timestamp());
  }

  // Skip moon calculations if still homing
  if(isHoming) {
    return;
  }

  // Normal lunar position calculations
  if (nowMillis - lastCheck >= 1000) {
    lastCheck = nowMillis;

    int index = getMoonDataIndex(now);
    if (index != lastIndexRead && now.minute() % INTERVAL_MINUTES == 0 && now.second() == 0) {
      lastIndexRead = index;

      float az, alt;
      readMoonDataAtTime(now, az, alt);

      Serial.println("Reading moon data! ");
      Serial.print("Azimuth: ");
      Serial.print(az);
      Serial.print(" | Altitude: ");
      Serial.println(alt);

      moveToMoonPosition(az, alt);
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
  // Add 90° offset for mechanical alignment
  float adjustedAz = fmod((azimuthDeg + AZIMUTH_OFFSET), 360.0);
  return (long)(adjustedAz * (stepper_azimuth_fullRotation / 360.0));
}

//If your physical mount maps altitude 0° to 0 steps, and increases as you go up:
long mapAltitudeToSteps(float altitudeDeg) {
  float altitudeRange = ALTITUDE_DEG_AT_FULL_STEPS - ALTITUDE_DEG_AT_ZERO_STEPS;
  float shifted = altitudeDeg - ALTITUDE_DEG_AT_ZERO_STEPS;
  return (long)(shifted * (stepper_altitude_fullRotation / altitudeRange));
}



//If instead 0° is horizontal and increases toward up, and you want 0° to map to 90° real altitude, then flip like:
// long mapAltitudeToSteps(float altitudeDeg) {
//   return (long)((90.0 - altitudeDeg) * (stepper_altitude_fullRotation / 360.0));
// }



