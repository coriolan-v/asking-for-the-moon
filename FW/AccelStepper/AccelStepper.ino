#include <AccelStepper.h>

#define stepPin 18
#define directionPin 17 //7
#define enPin 19

AccelStepper stepper(AccelStepper::DRIVER, stepPin, directionPin);

void setup()
{  
 // pinMode(HallSensor, INPUT_PULLUP);

  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, HIGH);
  // Change these to suit your stepper if you want
  
 stepper.setMaxSpeed(6000);
  stepper.setAcceleration(2000);
  stepper.moveTo(320000); // 32000 = 1 turn

}



void loop()
{
 stepper.run();

     
}





