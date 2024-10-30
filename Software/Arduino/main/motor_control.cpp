#include "motor_control.hpp"

motorControl::motorControl(int stepPin, int dirPin) : stepper(AccelStepper::DRIVER, stepPin, dirPin) {
    stepper.disableOutputs();
    stepper.setMaxSpeed(10000);
    stepper.setCurrentPosition(0);
    stepper.enableOutputs();
}

void motorControl::actuateMotors(long speed) {
  float stepperSpeed = speed * 1023.0 / 270;
  stepperSpeed = constrain(stepperSpeed, 0, 1023);
  stepper.setSpeed(stepperSpeed);
  stepper.runSpeed();
}
