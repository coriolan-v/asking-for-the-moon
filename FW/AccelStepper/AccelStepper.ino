#include <AccelStepper.h>
//#include <bluefruit.h>

#define stepPin 4
#define directionPin 5 //7
#define enPin 3

AccelStepper stepper(AccelStepper::DRIVER, stepPin, directionPin);

void setup()
{  
 // pinMode(HallSensor, INPUT_PULLUP);
Serial.begin(9600);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, HIGH);
  // Change these to suit your stepper if you want
  
 stepper.setMaxSpeed(2000);
  stepper.setAcceleration(500);
  stepper.moveTo(240000); // 32000 = 1 turn

}



void loop()
{
 stepper.run();

     
}





