#include "Arduino_interface.hpp"
#include "motor_control.hpp"

ArduinoInterface interface;

void setup() {
  Serial.begin(baudRate);
  Wire.begin(addr);
  Wire.onReceive([](int numBytes) { interface.receiveI2C(); });
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);
}

void loop() {
  motorControl motor1(stepPin, dirPin);
  motor1.actuateMotors(90);
}
