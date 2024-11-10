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

const int LimitSwitchMotor1 = 11;
const int LimitSwitchMotor2 = 12; //change pin
const int LimitSwitchMotor3 = 13; //change pin

class motorControl:public AccelStepper {
public:
    AccelStepper stepper;
    motorControl(int stepPin, int dirPin);
    void actuateMotors(long speed);
    void absoluteStepBlocked(long degrees);
    void relativeStepBlocked(long degrees);
    bool absoluteStepConcurrent(long degrees);
    long currentOrientation();
    
    static void moveInverseKinematics(std::vector<int>& inverseKinematics, motorControl& motor1, motorControl& motor2, motorControl& motor3);
    static void printPosition(motorControl& motor1, motorControl& motor2, motorControl& motor3);
    static void printSpeed(motorControl& motor1, motorControl& motor2, motorControl& motor3);
    static void homingSeqeunce(motorControl& motor1, motorControl& motor2, motorControl& motor3);
    static void homingSetup();
};

class parallelMotorControl:public MultiStepper { //make this inherit from motor control and move all static void functions to this class where it has three members set (motor1, motor2, motor3)
public: 
    MultiStepper steppers;
    void addAllSteppers(motorControl& motor1, motorControl& motor2, motorControl& motor3);
    void parallelMovement(std::vector<int>& inverseKinematics);
};
