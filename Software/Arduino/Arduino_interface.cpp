#include "Arduino_interface.hpp"

void ArduinoInterface::receiveI2C(int data) {
    while (Wire.available()) {
        int jointAngle = Wire.read();
        Serial.print("Joint angle: ");
        Serial.println(jointAngle);
        inverseKinematics.push_back(jointAngle);
  }
}