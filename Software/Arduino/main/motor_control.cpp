#include "motor_control.hpp"

motorControl::motorControl(int stepPin, int dirPin) : stepper(AccelStepper::DRIVER, stepPin, dirPin) { // constructor for each motor
    stepper.disableOutputs();
    stepper.setMaxSpeed(1000);
    stepper.setAcceleration(100);
    stepper.setCurrentPosition(0);
    stepper.enableOutputs();
}

long motorControl::currentOrientation() { // return motor position in degrees
  return (stepper.currentPosition())*360/3200;
}

void motorControl::actuateMotors(long stepperSpeed) { //moves motor continously at some speed
    stepperSpeed = constrain(stepperSpeed, 0, 10000);
    stepper.setSpeed(stepperSpeed);
    stepper.runSpeed();
}

void motorControl::absoluteStepBlocked(long degrees) { //moves motor to absolute position while blocking loop (clockwise/counterclockwise based off position)
    float stepperTarget = constrain(round(((degrees * 3200) / 360)), -3200, 3200);
    stepper.moveTo(stepperTarget);
    stepper.runToPosition();
}

void motorControl::relativeStepBlocked(long degrees) { //moves motor relative to position while blocking loop
    float stepperTarget = constrain(round(((degrees * 3200) / 360)), -3200, 3200);
    stepper.move(stepperTarget);
    stepper.runToPosition(); 
}

bool motorControl::absoluteStepConcurrent(long degrees) { //moves motor absolute to position without blocking loop (acceleration/deceleration)
    float stepperTarget = constrain(round(((degrees * 3200) / 360)), -3200, 3200);
    stepper.moveTo(stepperTarget);
    return stepper.run(); 
}

bool motorControl::absoluteConstantConcurrentStep(long degrees, long motorSpeed) { //moves motor absolute to position without blocking loop (constant speed)
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

void parallelMotorControl::moveInverseKinematics(std::vector<int>& inverseKinematics) { //move motors based off inverse kinematics (blocking)
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

void parallelMotorControl::setup() { //setup hardware necessary for homing sequence
    pinMode(LimitSwitchMotor1, INPUT);    
    //add setup for other limit switches
}

void parallelMotorControl::homingSequence() { //homing sequence
  while (digitalRead(LimitSwitchMotor1) != HIGH){ //make sure to check NO/NC for each limit switch
    motor1.actuateMotors(500); // make sure to check direction of speed for each motor
  }
  // Add IMU check to get phi offset angle for each stepper
}

void parallelMotorControl::printPosition() { //print position of all steppers to serial
    Serial.print("Position 1: ");
    Serial.print(motor1.currentOrientation());
    Serial.print(" Position 2: ");
    Serial.print(motor2.currentOrientation());
    Serial.print(" Position 3: ");
    Serial.println(motor3.currentOrientation());
}

void parallelMotorControl::printSpeed() { //print speed of all steppers to serial
    Serial.print("Speed 1: ");
    Serial.print(motor1.speed());
    Serial.print(" Speed 2: ");
    Serial.print(motor2.speed());
    Serial.print(" Speed 3: ");
    Serial.println(motor3.speed());
}
