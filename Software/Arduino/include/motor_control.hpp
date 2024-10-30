#include <AccelStepper.h>

class motorControl :: public AccelStepper {
public:
    motorControl(stepPin, dirPin);
    void actuateMotors(long speed);

private:
    AccelStepper stepper;
};