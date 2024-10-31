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

void parallelMotorControl::addAllSteppers(motorControl& motor1, motorControl& motor2, motorControl& motor3) {
  addStepper(motor1.stepper);
  addStepper(motor2.stepper);
  addStepper(motor3.stepper);
}

void parallelMotorControl::parallelMovement(std::vector<int> inverseKinematics) { //TODO: Change positions array to replace inverseKinematics vector in I2C interface
  if (inverseKinematics.empty())
    Serial.println("Nothing to move right now");
  else //move to positions the reset inverseKinematics so loop() wont continously move motors
  {
    long positions[3];
    positions[0] = inverseKinematics[1];
    positions[1] = inverseKinematics[2];
    positions[2] = inverseKinematics[3];
    moveTo(positions);
    runSpeedToPosition();
    inverseKinematics.clear();
    
  }
}