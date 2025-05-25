#include <AccelStepper.h>

#define stepPin 6
#define directionPin 7 //7
#define enPin 8

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

if(stepper.currentPosition() == 320000) stepper.moveTo(-320000); // 32000 = 1 turn
     
}





