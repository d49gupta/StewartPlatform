#include <AccelStepper.h>

const int dirPin = 2; //direction Pin
const int stepPin = 3; //pulse Pin
const int enPin = 4; //enable Pin

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin); //create instance of stepper

void setup() {
  Serial.begin(9600);
  stepper.disableOutputs(); //disable outputs initially
  stepper.setMaxSpeed(10000);
  stepper.setCurrentPosition(0); //zero current stepper position
  stepper.enableOutputs(); //enable outputs for motor
}

void actuateMotors(long speed){
  float stepperSpeed = degrees * 1023.0 / 270;
  stepperSpeed = constrain(stepperSpeed, 0, 1023);
  stepper.setSpeed(stepperSpeed);
  stepper.runSpeed();
}

void loop() {
 actuateMotors(90);
}
