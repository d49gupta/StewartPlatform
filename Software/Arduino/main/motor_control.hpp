#include <AccelStepper.h>

const int dirPin = 2;
const int stepPin = 3;
const int enPin = 4;

class motorControl:public AccelStepper {
public:
    motorControl(int stepPin, int dirPin);
    void actuateMotors(long speed);
    void absoluteStepBlocked(long degrees);
    void relativeStepBlocked(long degrees);

private:
    AccelStepper stepper;
};

class parallelMotorControl:public MultiStepper {
public:

private:
};
