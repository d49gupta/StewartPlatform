#include "Arduino_interface.hpp"
#include "motor_control.hpp"

ArduinoInterface interface;
motorControl motor1(stepPin1, dirPin1);
motorControl motor2(stepPin2, dirPin2);
motorControl motor3(stepPin3, dirPin3);   

void setup() {
  Serial.begin(interface.baudRate);
  interface.setup();
  Wire.onReceive([](int numBytes) { interface.receiveI2C(); });
}

void loop() {
    motor1.absoluteStepConcurrent(interface.inverseKinematics[1]);
    motor2.absoluteStepConcurrent(interface.inverseKinematics[2]);
    motor3.absoluteStepConcurrent(interface.inverseKinematics[3]);
    motorControl::printPosition(motor1, motor2, motor3);
}
