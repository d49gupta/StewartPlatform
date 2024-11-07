#include "Arduino_interface.hpp"
#include "motor_control.hpp"

ArduinoInterface interface;
motorControl motor1(stepPin1, dirPin1);
motorControl motor2(stepPin2, dirPin2);
motorControl motor3(stepPin3, dirPin3); 

void setup() {
  Serial.begin(baudRate);
  Wire.begin(addr);
  Wire.onReceive([](int numBytes) { interface.receiveI2C(); });
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);
}

void loop() {
  motorControl::moveInverseKinematics(interface.inverseKinematics, motor1, motor2, motor3);
}
