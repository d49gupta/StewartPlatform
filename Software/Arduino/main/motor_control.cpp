#include "motor_control.hpp"

motorControl::motorControl(int stepPin, int dirPin) : stepper(AccelStepper::DRIVER, stepPin, dirPin) {
    stepper.disableOutputs();
    stepper.setMaxSpeed(10000);
    stepper.setCurrentPosition(0);
    stepper.enableOutputs();
}

void motorControl::actuateMotors(long stepperSpeed) { //moves motor continously at some speed
  stepperSpeed = constrain(stepperSpeed, 0, 10000);
  stepper.setSpeed(stepperSpeed);
  stepper.runSpeed();
}

void motorControl::absoluteStepBlocked(long degrees) { //moves motor to absolute position
  long steps = degrees * 1023.0 / 270;
  stepper.moveTo(steps);
  stepper.runToPosition(); 
}

void motorControl::relativeStepBlocked(long degrees) { //moves motor relative to position
  long steps = degrees * 1023.0 / 270;
  stepper.move(steps);
  stepper.runToPosition(); 
}

