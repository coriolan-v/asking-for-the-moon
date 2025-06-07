#include "bluefruit.h"

#define stepper_azimuth_EN 19
#define stepper_azimuth_DIR 17
#define stepper_azimuth_STEP 18

#define stepper_altitude_EN A2
#define stepper_altitude_DIR 2
#define stepper_altitude_STEP 0


#include <AccelStepper.h>

// Define some steppers and the pins the will use
AccelStepper stepper_azimuth_bottom(AccelStepper::DRIVER, stepper_azimuth_STEP, stepper_azimuth_DIR);
AccelStepper stepper_altitude_top(AccelStepper::DRIVER, stepper_altitude_STEP, stepper_altitude_DIR);


void setup()
{  
    Serial.begin(9600);

    pinMode(stepper_azimuth_EN, OUTPUT);
    digitalWrite(stepper_azimuth_EN, HIGH);

    pinMode(stepper_altitude_EN, OUTPUT);
    digitalWrite(stepper_altitude_EN, HIGH);

    stepper_azimuth_bottom.setMaxSpeed(12000);
    stepper_azimuth_bottom.setAcceleration(2000);
    stepper_azimuth_bottom.moveTo(3200000); // 32000 = 1 turn

    stepper_altitude_top.setMaxSpeed(60000);
    stepper_altitude_top.setAcceleration(20000);
    stepper_altitude_top.moveTo(3200000); // 32000 = 1 turn
}

unsigned long previousMillis = 0;

void loop()
{
    stepper_azimuth_bottom.run();
    stepper_altitude_top.run();

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= 1000) {
        previousMillis = currentMillis;

        Serial.print("Azimuth Position: ");
        Serial.print(stepper_azimuth_bottom.currentPosition());
        Serial.print("\t"); // Tab spacing

        Serial.print("Altitude Position: ");
        Serial.println(stepper_altitude_top.currentPosition());
    }
}

