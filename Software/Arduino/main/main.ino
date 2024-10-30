#include "../include/Arduino_interface.hpp"
#include "../include/motor_control.hpp"

void setup() {
  Serial.begin(baudRate);
  Wire.begin(addr);
  Wire.onReceive(receiveI2C);

  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);
}

void loop() {
  delay(100);
  actuateMotors(90);
}
