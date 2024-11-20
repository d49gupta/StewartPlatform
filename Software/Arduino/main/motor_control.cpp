#include "motor_control.hpp"

motorControl::motorControl(int stepPin, int dirPin) : stepper(AccelStepper::DRIVER, stepPin, dirPin) {
    stepper.disableOutputs();
    stepper.setMaxSpeed(1000);
    stepper.setAcceleration(100);
    stepper.setCurrentPosition(0);
    stepper.enableOutputs();
    motorSpeedratio = 1;
}

long motorControl::currentOrientation() {
  return (stepper.currentPosition())*360/3200;
}

float motorControl::currentSpeed() {
  return (stepper.speed())*360/3200;
}

void motorControl::actuateMotors(long stepperSpeed) {
    stepperSpeed = constrain(stepperSpeed, -1000, 1000);
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

bool motorControl::absoluteConstantConcurrentStep(long degrees) {
    float stepperTarget = constrain(round(((degrees * 3200) / 360)), -3200, 3200);
    float stepperSpeed = constrain(round(motorSpeedratio * maxSpeed), 0, 1000);
    stepper.moveTo(stepperTarget);
    if (stepper.currentPosition() < stepperTarget) {
          stepper.setSpeed(abs(stepperSpeed)); //direction of speed based off target position
    } 
    else {
          stepper.setSpeed(-abs(stepperSpeed));
      }
    if (stepper.distanceToGo() != 0) { 
        stepper.runSpeed();
        return true;
    } 
    else {
        return false;
    }
}

void motorControl::setMotorPosition(long degrees) { 
  stepper.setCurrentPosition(0);
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
    pinMode(LimitSwitchMotor1.pin, INPUT_PULLUP);    
    pinMode(LimitSwitchMotor2.pin, INPUT_PULLUP);
    pinMode(LimitSwitchMotor3.pin, INPUT_PULLUP);
}

bool parallelMotorControl::homingSequence(int& ackBit) {
  while (LimitSwitchMotor1.state || LimitSwitchMotor2.state || LimitSwitchMotor3.state)
  {
    if (LimitSwitchMotor1.state)
      motor1.actuateMotors(-350);
    if (LimitSwitchMotor2.state)
      motor2.actuateMotors(-350);
    if (LimitSwitchMotor3.state)
      motor3.actuateMotors(-350);
      
    if (digitalRead(LimitSwitchMotor1.pin) == LOW)
      LimitSwitchMotor1.state = false;
    if (digitalRead(LimitSwitchMotor2.pin) == LOW )
      LimitSwitchMotor2.state = false;
    if (digitalRead(LimitSwitchMotor3.pin) == LOW )
      LimitSwitchMotor3.state = false;
  }
  Serial.println("Homing Sequence Completed");
  ackBit = 1;
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
    Serial.print(motor1.currentSpeed());
    Serial.print(" Speed 2: ");
    Serial.print(motor2.currentSpeed());
    Serial.print(" Speed 3: ");
    Serial.println(motor3.currentSpeed());
}

void parallelMotorControl::setAllMotorPositions(long degrees) {
    motor1.setMotorPosition(degrees);
    motor2.setMotorPosition(degrees);
    motor3.setMotorPosition(degrees);
    printPosition();
}

void parallelMotorControl::calculateSpeed(std::vector<int> inverseKinematics) {
    int motor1_distance = abs(motor1.currentOrientation() - inverseKinematics[0]);
    int motor2_distance = abs(motor2.currentOrientation() - inverseKinematics[1]);
    int motor3_distance = abs(motor3.currentOrientation() + inverseKinematics[2]);
    
    int max_distance = std::max(motor1_distance, std::max(motor2_distance, motor3_distance));

    if (max_distance == 0) {
      motor1.motorSpeedratio = 0;
      motor2.motorSpeedratio = 0;
      motor3.motorSpeedratio = 0;
      return;
    }

    motor1.motorSpeedratio = (static_cast<float>(motor1_distance) / max_distance);
    motor2.motorSpeedratio = (static_cast<float>(motor2_distance) / max_distance);
    motor3.motorSpeedratio = (static_cast<float>(motor3_distance) / max_distance);
}
