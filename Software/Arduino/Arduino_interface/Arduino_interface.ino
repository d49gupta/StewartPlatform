#include <Wire.h>
#include <ArduinoSTL.h>
#include <vector>

const int SDA_Pin = 20; // TODO: Make all consts in header file
const int SCL_Pin = 21;

std::vector<int> inverseKinematics;
int addr = 0x8;

void receiveEvent(int howMany) {
  while (Wire.available()) {
    int jointAngle = Wire.read();
    Serial.print("Joint angle: ");
    Serial.println(jointAngle);
    inverseKinematics.push_back(jointAngle);
  }
}


void setup() {
  Serial.begin(9600);
  Wire.begin(addr);
  Wire.onReceive(receiveEvent);

  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);
}

void loop() {
  delay(100); // Keep waiting for data
}
