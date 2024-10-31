#include <AccelStepper.h>
#include <MultiStepper.h>
#include <vector>

const int dirPin = 2;
const int stepPin = 3;
const int enPin = 4;

class motorControl:public AccelStepper {
public:
    AccelStepper stepper;
    motorControl(int stepPin, int dirPin);
    void actuateMotors(long speed);
    void absoluteStepBlocked(long degrees);
    void relativeStepBlocked(long degrees);
};

class parallelMotorControl:public MultiStepper {
public: 
    void addAllSteppers(motorControl& motor1, motorControl& motor2, motorControl& motor3);
    void parallelMovement(std::vector<int> inverseKinematics);
};
