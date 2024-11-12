#include <AccelStepper.h>
#include <MultiStepper.h>
#include <vector>

const int dirPin1 = 2;
const int stepPin1 = 3;
const int enPin1 = 4;

const int dirPin2 = 5;
const int stepPin2 = 6;
const int enPin2 = 7;

const int dirPin3 = 8;
const int stepPin3 = 9;
const int enPin3 = 10;

class motorControl:public AccelStepper {
public:
    AccelStepper stepper;
    static const long defaultSpeed = 500;

    motorControl() {} // default constructor
    motorControl(int stepPin, int dirPin); // constructor
    void actuateMotors(long speed); // moves motor continously at some speed
    void absoluteStepBlocked(long degrees); // moves motor to absolute position while blocking loop (clockwise/counterclockwise based off position)
    void relativeStepBlocked(long degrees); // moves motor relative to position while blocking loop
    bool absoluteStepConcurrent(long degrees); // moves motor absolute to position without blocking loop (acceleration/deceleration)
    bool absoluteConstantConcurrentStep(long degrees, long motorSpeed = defaultSpeed); // moves motor absolute to position without blocking loop (constant speed)
    long currentOrientation(); // return motor position in degrees
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

private:
    struct LimitSwitch {
      int pin;
      bool state;
    };
    
    LimitSwitch LimitSwitchMotor1 = {11, true}; // LimitSwitchmotor1 is 1(HIGH) when not pressed, NC
    LimitSwitch LimitSwitchMotor2 = {12, true}; // LimitSwitchmotor2 is 1(HIGH) when not pressed, NC
    LimitSwitch LimitSwitchMotor3 = {13, true}; // LimitSwitchmotor3 is 1(HIGH) when not pressed, NC
};
