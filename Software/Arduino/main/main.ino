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
  Wire.onRequest([](int numBytes) { interface.sendI2C(); });
}

void loop() {
    motor1.absoluteConstantConcurrentStep(interface.inverseKinematics[0]);
}
