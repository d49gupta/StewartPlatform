#include "Arduino_interface.hpp"
#include "motor_control.hpp"

ArduinoInterface interface;
motorControl motor1(stepPin, dirPin);

void setup() {
  Serial.begin(baudRate);
  Wire.begin(addr);
  Wire.onReceive([](int numBytes) { interface.receiveI2C(); });
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);
}

void loop() {
  motor1.absoluteStepBlocked(90);
  delay(10000);
  motor1.relativeStepBlocked(115);
  delay(10000); //delay 10 seconds to check results
}
