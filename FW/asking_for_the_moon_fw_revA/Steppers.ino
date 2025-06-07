#include <AccelStepper.h>

#define stepper_azimuth_EN 19
#define stepper_azimuth_DIR 17
#define stepper_azimuth_STEP 18
#define stepper_azimuth_Home A0

#define stepper_azimuth_speed 12000
#define stepper_azimuth_acc 2000
const long stepper_azimuth_fullRotation = 240000;

#define stepper_altitude_EN A2
#define stepper_altitude_DIR 2
#define stepper_altitude_STEP 0
#define stepper_altitude_Home A1

#define stepper_altitude_speed 60000
#define stepper_altitude_acc 20000
const long stepper_altitude_fullRotation = 240000;

// Define some steppers and the pins the will use
AccelStepper stepper_azimuth_bottom(AccelStepper::DRIVER, stepper_azimuth_STEP, stepper_azimuth_DIR);
AccelStepper stepper_altitude_top(AccelStepper::DRIVER, stepper_altitude_STEP, stepper_altitude_DIR);

bool hasHomedToday = false;

void setupSteppers() {
  pinMode(stepper_azimuth_EN, OUTPUT);
  digitalWrite(stepper_azimuth_EN, HIGH);

  pinMode(stepper_altitude_EN, OUTPUT);
  digitalWrite(stepper_altitude_EN, HIGH);

  stepper_azimuth_bottom.setMaxSpeed(12000);
  stepper_azimuth_bottom.setAcceleration(2000);
  stepper_azimuth_bottom.moveTo(3200000);  // 32000 = 1 turn

  stepper_altitude_top.setMaxSpeed(60000);
  stepper_altitude_top.setAcceleration(20000);
  stepper_altitude_top.moveTo(3200000);  // 32000 = 1 turn
}

void homeSteppers() {
  findHomeAzimuth();
  findHomeAltitude();
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

  stepper_azimuth_bottom.moveTo(azSteps);
  stepper_altitude_top.moveTo(altSteps);

  Serial.print("New Target → Azimuth: ");
  Serial.print(azSteps);
  Serial.print(" | Altitude: ");
  Serial.println(altSteps);
}

void runSteppersWithStatus() {
  bool azMoving = stepper_azimuth_bottom.distanceToGo() != 0;
  bool altMoving = stepper_altitude_top.distanceToGo() != 0;

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

  // Keep moving the motors
  stepper_azimuth_bottom.run();
  stepper_altitude_top.run();
}

void checkDailyHoming(DateTime now) {
  // Trigger homing at 08:00
  if (now.hour() == 8 && now.minute() == 0) {
    if (!hasHomedToday) {
      Serial.println("🔄 Performing daily homing...");
      homeSteppers();
      hasHomedToday = true;
    }
  } else {
    // Reset flag after 08:00 so we can trigger again the next day
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