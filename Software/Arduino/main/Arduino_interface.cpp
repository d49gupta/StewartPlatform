#include "Arduino_interface.hpp"

void ArduinoInterface::receiveI2C() { //receive I2C data from rpi and save numbers in vector
    while (Wire.available()) {
        int jointAngle = Wire.read();
        Serial.print("Joint angle: ");
        Serial.println(jointAngle);
        inverseKinematics.push_back(jointAngle);
  }
}
