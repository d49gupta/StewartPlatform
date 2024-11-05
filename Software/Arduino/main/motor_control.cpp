#include "motor_control.hpp"

motorControl::motorControl(int stepPin, int dirPin) : stepper(AccelStepper::DRIVER, stepPin, dirPin) { // constructor for each motor
    stepper.disableOutputs();
    stepper.setMaxSpeed(10000);
    stepper.setSpeed(750);
    stepper.setAcceleration(100);
    stepper.setCurrentPosition(0);
    stepper.enableOutputs();
}

void motorControl::actuateMotors(long stepperSpeed) { //moves motor continously at some speed
  stepperSpeed = constrain(stepperSpeed, 0, 10000);
  stepper.setSpeed(stepperSpeed);
  stepper.runSpeed();
}

void motorControl::absoluteStepBlocked(long degrees) { //moves motor to absolute position while blocking loop
  float stepperTarget = constrain(round(((degrees * 3200) / 360)), -3200, 3200); //TODO: Find constraints of steps of steppers (270 degrees?)
  stepper.moveTo(stepperTarget);
  stepper.runToPosition(); 
}

void motorControl::relativeStepBlocked(long degrees) { //moves motor relative to position while blocking loop
  float stepperTarget = constrain(round(((degrees * 3200) / 360)), -3200, 3200); //TODO: Find constraints of steps of steppers (270 degrees?)
  stepper.move(stepperTarget);
  stepper.runToPosition(); 
}

void motorControl::absoluteStepConcurrent(long degrees) { //moves motor absolute to position without blocking loop
  float stepperTarget = constrain(round(((degrees * 3200) / 360)), -3200, 3200); //TODO: Find constraints of steps of steppers (270 degrees?)
  stepper.moveTo(stepperTarget);
  stepper.run(); 
}

void parallelMotorControl::addAllSteppers(motorControl& motor1, motorControl& motor2, motorControl& motor3) {
  steppers.addStepper(motor1.stepper);
  steppers.addStepper(motor2.stepper);
  steppers.addStepper(motor3.stepper);
}

void parallelMotorControl::parallelMovement(std::vector<int>& inverseKinematics) { //TODO: Change positions array to replace inverseKinematics vector in I2C interface
  if (inverseKinematics.empty())
    Serial.println("Nothing to move right now");
  else //move to positions the reset inverseKinematics so loop() wont continously move motors
  {
    long positions[3];
    positions[0] = inverseKinematics[1];
    positions[1] = inverseKinematics[2];
    positions[2] = inverseKinematics[3];
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    inverseKinematics.clear();
  }
}
