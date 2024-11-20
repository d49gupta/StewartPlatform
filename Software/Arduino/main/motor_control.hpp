#include <AccelStepper.h>
#include <MultiStepper.h>
#include <vector>
#pragma once

class motorControl:public AccelStepper {
public:
    AccelStepper stepper;
    static const long maxSpeed = 500;
    float motorSpeedratio;

    motorControl() {} // default constructor
    motorControl(int stepPin, int dirPin); // constructor
    void actuateMotors(long speed); // moves motor continously at some speed
    void absoluteStepBlocked(long degrees); // moves motor to absolute position while blocking loop (clockwise/counterclockwise based off position)
    void relativeStepBlocked(long degrees); // moves motor relative to position while blocking loop
    bool absoluteStepConcurrent(long degrees); // moves motor absolute to position without blocking loop (acceleration/deceleration)
    bool absoluteConstantConcurrentStep(long degrees); // moves motor absolute to position without blocking loop (constant speed)
    long currentOrientation(); // return motor position in degrees
    float currentSpeed(); // return motor speed in degrees/s
    void setMotorPosition(long degrees); // wrapper for setCurrentPosition()
};

class parallelMotorControl:public motorControl {
public: 
    motorControl& motor1;
    motorControl& motor2;
    motorControl& motor3;

    parallelMotorControl(motorControl& m1, motorControl& m2, motorControl& m3) : motor1(m1), motor2(m2), motor3(m3) {} // constructor that uses references of existing motors
    void moveInverseKinematics(std::vector<int>& inverseKinematics); //move motors based off inverse kinematics (blocking)
    void printPosition(); // print position of all steppers to serial
    void printSpeed(); // print speed of all steppers to serial
    bool homingSequence(int& ackBit); // homing sequence for motors
    void setup(); // hardware setup (limit switches)
    void setAllMotorPositions(long degrees); // set all motor positions
    void calculateSpeed(std::vector<int> inverseKinematics); //calculate speed to make motors achieve position at same time

private:
    struct LimitSwitch {
      int pin;
      bool state;
    };
    
    LimitSwitch LimitSwitchMotor1 = {11, true}; // LimitSwitchmotor1 is 1(HIGH) when not pressed, NC
    LimitSwitch LimitSwitchMotor2 = {12, true}; // LimitSwitchmotor2 is 1(HIGH) when not pressed, NC
    LimitSwitch LimitSwitchMotor3 = {13, true}; // LimitSwitchmotor3 is 1(HIGH) when not pressed, NC
};
