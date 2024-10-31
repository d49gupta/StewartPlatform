#include "Arduino_interface.hpp"
#include "motor_control.hpp"

ArduinoInterface interface;
motorControl motor1(stepPin, dirPin);

// uncomment this section to try and run all motors concurrently
//parallelMotorControl allSteppers;
//motorControl motor2(stepPin, dirPin); //change pins if needed
//motorControl motor3(stepPin, dirPin); //change pins if needed


void setup() {
  Serial.begin(baudRate);
  Wire.begin(addr);
  Wire.onReceive([](int numBytes) { interface.receiveI2C(); });
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);

  //uncomment to try and run all motors concurrently
//  allSteppers.addAllSteppers(motor1, motor2, motor3);
}

void loop() {
  motor1.absoluteStepBlocked(90);
  delay(10000); //delay 10 seconds to check results
  motor1.relativeStepBlocked(115);
  delay(10000);

  // uncomment this section and comment section right above to try and run all motors concurrently
//   allSteppers.parallelMovement(interface.inverseKinematics);
//   delay(10000);
}
