#include "Arduino_interface.hpp"

void ArduinoInterface::setup() {
  Wire.begin(addr);
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);
}

void ArduinoInterface::receiveI2C() {
  inverseKinematics.clear();
  if (Wire.available()) {
    int command = Wire.read();

    if (command == 0) {
      while (Wire.available()) {
          int jointAngle = Wire.read();
          Serial.print("Joint angle: ");
          Serial.println(jointAngle);
          inverseKinematics.push_back(jointAngle);
      }
    }
    else {
      Serial.println("Shutting Down for 100 Seconds");
      delay(100000);
    }
  }
}

void ArduinoInterface::sendI2C() {
  Wire.write(datatoSend);
}