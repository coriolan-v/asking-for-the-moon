#include <AccelStepper.h>

#define stepper_azimuth_EN 3
#define stepper_azimuth_DIR 5
#define stepper_azimuth_STEP 4
#define stepper_azimuth_Home A4

#define stepper_azimuth_speed 3000
#define stepper_azimuth_acc 500
const long stepper_azimuth_fullRotation = 240000;

#define stepper_altitude_EN A5
#define stepper_altitude_DIR 1
#define stepper_altitude_STEP 2
#define stepper_altitude_Home A6

#define stepper_altitude_speed 3000
#define stepper_altitude_acc 500
const long stepper_altitude_fullRotation = 320000;

// Define some steppers and the pins the will use
AccelStepper stepper_azimuth_bottom(AccelStepper::DRIVER, stepper_azimuth_STEP, stepper_azimuth_DIR);
AccelStepper stepper_altitude_top(AccelStepper::DRIVER, stepper_altitude_STEP, stepper_altitude_DIR);

bool hasHomedToday = false;

bool azHomed = false;
bool altHomed = false;

unsigned long homingPrintTimer = 0;

unsigned long homingStartTime = 0;
const unsigned long HOMING_TIMEOUT_MS = 540000; // 3 minutes

void setupSteppers() {
  pinMode(stepper_azimuth_EN, OUTPUT);
  digitalWrite(stepper_azimuth_EN, HIGH);

  pinMode(stepper_altitude_EN, OUTPUT);
  digitalWrite(stepper_altitude_EN, HIGH);

  stepper_azimuth_bottom.setMaxSpeed(stepper_azimuth_speed);
  stepper_azimuth_bottom.setAcceleration(stepper_azimuth_acc);
  stepper_azimuth_bottom.moveTo(stepper_azimuth_fullRotation);  // 32000 = 1 turn

  stepper_altitude_top.setMaxSpeed(stepper_altitude_speed);
  stepper_altitude_top.setAcceleration(stepper_altitude_acc);
  stepper_altitude_top.moveTo(stepper_altitude_fullRotation);  // 32000 = 1 turn
}

void homeSteppers() {
  isHoming = true;
  azHomed = false;
  altHomed = false;
  homingPrintTimer = 0;
  homingStartTime = millis();  // ✅ Start timeout clock

  Serial.println("🔄 Homing sequence started...");

  if (analogRead(stepper_azimuth_Home) >= 700) {
    stepper_azimuth_bottom.setCurrentPosition(0);
    azHomed = true;
    Serial.println("✅ Azimuth already at home.");
  } else {
    stepper_azimuth_bottom.setCurrentPosition(0);
    stepper_azimuth_bottom.move(stepper_azimuth_fullRotation);
  }

  if (analogRead(stepper_altitude_Home) >= 700) {
    stepper_altitude_top.setCurrentPosition(0);
    altHomed = true;
    Serial.println("✅ Altitude already at home.");
  } else {
    stepper_altitude_top.setCurrentPosition(0);
    stepper_altitude_top.move(stepper_altitude_fullRotation);
  }
}



void updateHoming() {
  if (!isHoming) return;

  unsigned long now = millis();
  bool stepTaken = false;

  // Always run both motors regardless of state
  if (!azHomed) {
    stepper_azimuth_bottom.run();
    stepTaken = true;

    if (analogRead(stepper_azimuth_Home) >= 700) {
      stepper_azimuth_bottom.stop();
      stepper_azimuth_bottom.setCurrentPosition(0);
      azHomed = true;
      Serial.println("✅ Azimuth homed.");
    }
  }

  if (!altHomed) {
    stepper_altitude_top.run();
    stepTaken = true;

    if (analogRead(stepper_altitude_Home) >= 700) {
      stepper_altitude_top.stop();
      stepper_altitude_top.setCurrentPosition(0);
      altHomed = true;
      Serial.println("✅ Altitude homed.");
    }
  }

  // Print status once per second
  if (now - homingPrintTimer >= 1000) {
    homingPrintTimer = now;
    if (!azHomed) Serial.println("HOMING AZIMUTH...");
    if (!altHomed) Serial.println("HOMING ALTITUDE...");
  }

  // Timeout logic
  if (!azHomed || !altHomed) {
    if (now - homingStartTime >= HOMING_TIMEOUT_MS) {
      Serial.println("⚠️ Homing timeout reached. Forcing home...");

      if (!azHomed) {
        stepper_azimuth_bottom.stop();
        stepper_azimuth_bottom.setCurrentPosition(0);
        azHomed = true;
        Serial.println("⚠️ Azimuth forced home.");
      }

      if (!altHomed) {
        stepper_altitude_top.stop();
        stepper_altitude_top.setCurrentPosition(0);
        altHomed = true;
        Serial.println("⚠️ Altitude forced home.");
      }
    }
  }

  if (azHomed && altHomed) {
    isHoming = false;
    Serial.println("🏁 Homing complete.");

    DateTime now = rtc.now();
    float az, alt;
    readMoonDataAtTime(now, az, alt);
    moveToMoonPosition(az, alt);
    lastIndexRead = -1;
  }
}





unsigned long previousMillis = 0;
unsigned long previousMillisAzimuth = 0;
unsigned long previousMillisAltitude = 0;
const long interval = 1000;  // Print every second

void findHomeAzimuth() {
  stepper_azimuth_bottom.setCurrentPosition(0);               // Reset position tracking
  stepper_azimuth_bottom.move(stepper_azimuth_fullRotation);  // Move at least one full rotation
  stepper_azimuth_bottom.run();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisAzimuth >= interval) {
    previousMillisAzimuth = currentMillis;
    Serial.println("HOMING AZIMUTH...");
  }

  if (analogRead(stepper_azimuth_Home) >= 700) {  // Home sensor triggered
    stepper_azimuth_bottom.stop();
    stepper_azimuth_bottom.setCurrentPosition(0);
    Serial.println("Azimuth home position set.");
  }
}

void findHomeAltitude() {
  stepper_altitude_top.setCurrentPosition(0);                // Reset position tracking
  stepper_altitude_top.move(stepper_altitude_fullRotation);  // Move at least one full rotation
  stepper_altitude_top.run();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisAltitude >= interval) {
    previousMillisAltitude = currentMillis;
    Serial.println("HOMING ALTITUDE...");
  }

  if (analogRead(stepper_altitude_Home) >= 700) {  // Home sensor triggered
    stepper_altitude_top.stop();
    stepper_altitude_top.setCurrentPosition(0);
    Serial.println("Altitude home position set.");
  }
}

void runSteppers() {
  stepper_azimuth_bottom.run();
  stepper_altitude_top.run();
}

void moveToMoonPosition(float azimuthDeg, float altitudeDeg) {
  long azSteps = mapAzimuthToSteps(azimuthDeg);
  long altSteps = mapAltitudeToSteps(altitudeDeg);

  if (stepper_azimuth_bottom.targetPosition() != azSteps)
    stepper_azimuth_bottom.moveTo(azSteps);
  if (stepper_altitude_top.targetPosition() != altSteps)
    stepper_altitude_top.moveTo(altSteps);

  Serial.print("New Target → Azimuth: ");
  Serial.print(azSteps);
  Serial.print(" | Altitude: ");
  Serial.println(altSteps);
}


void runSteppersWithStatus() {
  static unsigned long lastStatusPrint = 0;
  const unsigned long statusInterval = 1000;

  bool azMoving = stepper_azimuth_bottom.distanceToGo() != 0;
  bool altMoving = stepper_altitude_top.distanceToGo() != 0;

  unsigned long now = millis();
  if (now - lastStatusPrint >= statusInterval) {
    lastStatusPrint = now;

    if (azMoving) {
      Serial.print("AZIMUTH moving... current: ");
      Serial.print(stepper_azimuth_bottom.currentPosition());
      Serial.print(" / target: ");
      Serial.println(stepper_azimuth_bottom.targetPosition());
    }

    if (altMoving) {
      Serial.print("ALTITUDE moving... current: ");
      Serial.print(stepper_altitude_top.currentPosition());
      Serial.print(" / target: ");
      Serial.println(stepper_altitude_top.targetPosition());
    }
  }

  stepper_azimuth_bottom.run();
  stepper_altitude_top.run();
}


void checkDailyHoming(DateTime now) {
  if (now.hour() == 8 && now.minute() == 0) {
    if (!hasHomedToday && !isHoming) {
      homeSteppers();  // Triggers non-blocking homing
      hasHomedToday = true;
    }
  } else {
    hasHomedToday = false;
  }
}


unsigned long lastLoopCheck= 0;
void homeEveryDay() 
{
  if (millis() - lastLoopCheck >= 1000) {
    lastLoopCheck = millis();

    DateTime now = rtc.now();

    checkDailyHoming(now);  // ← Re-home once per day at 08:00
  }
}