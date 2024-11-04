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
   motor1.relativeStepBlocked(90);
   motor2.relativeStepBlocked(90);
   motor3.relativeStepBlocked(90);
}
