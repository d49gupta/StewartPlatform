#include "Arduino_interface.hpp"
#include "motor_control.hpp"

ArduinoInterface interface;
motorControl motor1(stepPin1, dirPin1);
motorControl motor2(stepPin2, dirPin2);
motorControl motor3(stepPin3, dirPin3); 
parallelMotorControl parallelController(motor1, motor2, motor3);

void setup() {
  Serial.begin(interface.baudRate);
  interface.setup();
  Wire.onReceive([](int numBytes) { interface.receiveI2C(); });
  Wire.onRequest(ArduinoInterface::sendI2C);

}

void loop() {
   motor1.absoluteConstantConcurrentStep(interface.inverseKinematics[0]);
   motor2.absoluteConstantConcurrentStep(interface.inverseKinematics[1]);
   motor3.absoluteConstantConcurrentStep(interface.inverseKinematics[2]);
}
