#include <Wire.h>
#include <ArduinoSTL.h>
#include <vector>

const int SDA_Pin = 20; //TODO: Make all consts in header file
const int SCL_Pin = 21;
const int ledPin = 13;

std::vector<float> inverseKinematics;
int addr = 0x8;
 
void setup() {
  Wire.begin(addr);
  Wire.onReceive(receiveEvent);
  
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);
}
 
void receiveEvent(int inverseKinematics) { //TODO: Make everything into class with each motor as object
  while (Wire.available()) { 
    float jointAngle = wire.read();
    inverseKinematics.push_back(jointAngle);
  }
}

void loop() {
  delay(100); // Keep waiting for data
}

