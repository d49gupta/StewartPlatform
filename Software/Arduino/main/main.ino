#include "Arduino_interface.hpp"
#include "motor_control.hpp"

ArduinoInterface interface;
motorControl motor1(stepPin1, dirPin1);
motorControl motor2(stepPin2, dirPin2);
motorControl motor3(stepPin3, dirPin3); 
parallelMotorControl parallelController(motor1, motor2, motor3);
int ArduinoInterface::datatoSend = 0;

void setup() {
    Serial.begin(interface.baudRate);
    interface.setup();
    Wire.onReceive([](int numBytes) { interface.receiveI2C(); });
    Wire.onRequest(ArduinoInterface::sendI2C);
    parallelController.setup();
}

void loop() {
    if (interface.getStartProgram()) { // wait until rpi requests program to begin calibration
        Serial.println("Program has started");
        bool calibrationStatus = parallelController.homingSequence(ArduinoInterface::datatoSend);
        interface.setStartProgram(false);
        parallelController.setAllMotorPositions(0);
        // do relative movement here with IMU
        parallelController.setAllMotorPositions(0);
        interface.setCalibrationStatus(true);
    }
    
    if (interface.getCalibrationStatus()) { // wait until calibration status has completed
        parallelController.moveInverseKinematics(interface.inverseKinematics);
    }
}
