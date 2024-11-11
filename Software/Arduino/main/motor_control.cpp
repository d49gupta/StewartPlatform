#include "motor_control.hpp"

motorControl::motorControl(int stepPin, int dirPin) : stepper(AccelStepper::DRIVER, stepPin, dirPin) {
    stepper.disableOutputs();
    stepper.setMaxSpeed(1000);
    stepper.setAcceleration(100);
    stepper.setCurrentPosition(0);
    stepper.enableOutputs();
}

long motorControl::currentOrientation() {
  return (stepper.currentPosition())*360/3200;
}

void motorControl::actuateMotors(long stepperSpeed) {
    stepperSpeed = constrain(stepperSpeed, 0, 10000);
    stepper.setSpeed(stepperSpeed);
    stepper.runSpeed();
}

void motorControl::absoluteStepBlocked(long degrees) {
    float stepperTarget = constrain(round(((degrees * 3200) / 360)), -3200, 3200);
    stepper.moveTo(stepperTarget);
    stepper.runToPosition();
}

void motorControl::relativeStepBlocked(long degrees) {
    float stepperTarget = constrain(round(((degrees * 3200) / 360)), -3200, 3200);
    stepper.move(stepperTarget);
    stepper.runToPosition(); 
}

bool motorControl::absoluteStepConcurrent(long degrees) {
    float stepperTarget = constrain(round(((degrees * 3200) / 360)), -3200, 3200);
    stepper.moveTo(stepperTarget);
    return stepper.run(); 
}

bool motorControl::absoluteConstantConcurrentStep(long degrees, long motorSpeed) {
    float stepperTarget = constrain(round(((degrees * 3200) / 360)), -3200, 3200);
    stepper.moveTo(stepperTarget);
    if (stepper.currentPosition() < stepperTarget) {
          stepper.setSpeed(abs(motorSpeed)); //direction of speed based off target position
    } 
    else {
          stepper.setSpeed(-abs(motorSpeed));
      }
    if (stepper.distanceToGo() != 0) { 
        stepper.runSpeed();
        return true;
    } 
    else {
        return false;
    }
}

void parallelMotorControl::moveInverseKinematics(std::vector<int>& inverseKinematics) {
  if (inverseKinematics.empty()) {
    Serial.println("Nothing to move right now");
    printPosition();
    printSpeed();
    return;
  }
  while (motor1.absoluteConstantConcurrentStep(inverseKinematics[1]) && motor2.absoluteConstantConcurrentStep(inverseKinematics[2]) && motor3.absoluteConstantConcurrentStep(inverseKinematics[3])) {
    printPosition();
    printSpeed();
  }
}

void parallelMotorControl::setup() {
    pinMode(LimitSwitchMotor1, INPUT_PULLUP);    
    //add setup for other limit switches
    pinMode(LimitSwitchMotor2, INPUT_PULLUP);
    pinMode(LimitSwitchMotor3, INPUT_PULLUP);
}

void parallelMotorControl::homingSequence() {
  while (digitalRead(LimitSwitchMotor1) == HIGH ){ //make sure to check NO/NC for each limit switch
    motor1.actuateMotors(1000); // make sure to check direction of speed for each motor
  }
  while (digitalRead(LimitSwitchMotor2) == HIGH) { 
    motor2.actuateMotors(500); // make sure to check direction of speed for each motor
  }
    while (digitalRead(LimitSwitchMotor3) == HIGH) { 
    motor3.actuateMotors(500); // make sure to check direction of speed for each motor
  }
  
}

void parallelMotorControl::printPosition() {
    Serial.print("Position 1: ");
    Serial.print(motor1.currentOrientation());
    Serial.print(" Position 2: ");
    Serial.print(motor2.currentOrientation());
    Serial.print(" Position 3: ");
    Serial.println(motor3.currentOrientation());
}

void parallelMotorControl::printSpeed() {
    Serial.print("Speed 1: ");
    Serial.print(motor1.speed());
    Serial.print(" Speed 2: ");
    Serial.print(motor2.speed());
    Serial.print(" Speed 3: ");
    Serial.println(motor3.speed());
}
