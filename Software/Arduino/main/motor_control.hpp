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
    motorControl(int stepPin, int dirPin);
    void actuateMotors(long speed);
    void absoluteStepBlocked(long degrees);
    void relativeStepBlocked(long degrees);
    bool absoluteStepConcurrent(long degrees);
    bool absoluteConstantConcurrentStep(long degrees, long motorSpeed = defaultSpeed);
    long currentOrientation();
};

class parallelMotorControl:public motorControl { //make this inherit from motor control and move all static void functions to this class where it has three members set (motor1, motor2, motor3)
public: 
    motorControl& motor1;
    motorControl& motor2;
    motorControl& motor3;

    parallelMotorControl(motorControl& m1, motorControl& m2, motorControl& m3) : motor1(m1), motor2(m2), motor3(m3) {}
    void moveInverseKinematics(std::vector<int>& inverseKinematics);
    void printPosition();
    void printSpeed();
    void homingSequence();
    void setup();

private:
    const int LimitSwitchMotor1 = 11;
    const int LimitSwitchMotor2 = 12;
    const int LimitSwitchMotor3 = 13;
};
