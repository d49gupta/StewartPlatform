#include "Arduino_interface.hpp"
#include "motor_control.hpp"

const int dirPin1 = 2;
const int stepPin1 = 3;
const int enPin1 = 4;

const int dirPin2 = 5;
const int stepPin2 = 6;
const int enPin2 = 7;

const int dirPin3 = 8;
const int stepPin3 = 9;
const int enPin3 = 10;

motorControl motor1(stepPin1, dirPin1);
motorControl motor2(stepPin2, dirPin2);
motorControl motor3(stepPin3, dirPin3); 
parallelMotorControl parallelController(motor1, motor2, motor3);
ArduinoInterface interface(motor1, motor2, motor3);
int ArduinoInterface::datatoSend = 0;

void setup() {
    Serial.begin(interface.baudRate);
    interface.setup();
    Wire.onReceive([](int numBytes) { interface.receiveI2C(); });
    Wire.onRequest(ArduinoInterface::sendI2C);
    parallelController.setup();
}

void loop() {
//    if (interface.getStartProgram()) { // wait until rpi requests program to begin calibration
//        Serial.println("Program has started");
//        bool calibrationStatus = parallelController.homingSequence(ArduinoInterface::datatoSend);
//        interface.setStartProgram(false);
//        parallelController.setAllMotorPositions(0);
//        interface.setCalibrationStatus(true);
//    }
//    if (interface.getCalibrationStatus()) {
      motor1.absoluteConstantConcurrentStep(interface.inverseKinematics[0]);
      motor2.absoluteConstantConcurrentStep(interface.inverseKinematics[1]);
      motor3.absoluteConstantConcurrentStep(interface.inverseKinematics[2]);
//    }

}
