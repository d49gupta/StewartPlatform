#include "../Arduino_interface.hpp"

void setup() {
  Serial.begin(baudRate);
  Wire.begin(addr);
  Wire.onReceive(receiveI2C);

  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);
}

void loop() {
  delay(100);
}
