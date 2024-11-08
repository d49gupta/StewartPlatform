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
    motorControl(int stepPin, int dirPin);
    void actuateMotors(long speed);
    void absoluteStepBlocked(long degrees);
    void relativeStepBlocked(long degrees);
    void absoluteStepConcurrent(long degrees);
    static void moveInverseKinematics(std::vector<int>& inverseKinematics, motorControl& motor1, motorControl& motor2, motorControl& motor3);
    long currentOrientation();
protected:
    void calibration();
    long targetPosition; //in degrees
};

class parallelMotorControl:public MultiStepper {
public: 
    MultiStepper steppers;
    void addAllSteppers(motorControl& motor1, motorControl& motor2, motorControl& motor3);
    void parallelMovement(std::vector<int>& inverseKinematics);
};
