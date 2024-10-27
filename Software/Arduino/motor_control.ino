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

void actuateMotors(float degrees){
  float target = degrees * 1023.0 / 270;
  target = constrain(target, 0, 1023);
  stepper.move(target);
}

void loop() {
  actuateMotors(90.0);
}