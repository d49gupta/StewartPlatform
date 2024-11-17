#include <Wire.h>
#include <ArduinoSTL.h>
#include <vector>
#include "motor_control.hpp"

class ArduinoInterface {
public: 
    const int baudRate = 9600;
    std::vector<int> inverseKinematics = {0, 0, 0};
    static int datatoSend;
    
    ArduinoInterface(motorControl& m1, motorControl& m2, motorControl& m3) : motorController(m1, m2, m3){} // constructor to set constructor of parallelMotorControll instance
    void setup(); // setup I2C pins and master address
    void receiveI2C(); // receive I2C data from rpi and save numbers in vector or for E-STOP
    static void sendI2C();  // send I2C data from arduino to rpi
    bool getStartProgram(); // get private member startProgram
    void setStartProgram(bool value); // set value of private member startProgram
    bool getCalibrationStatus(); // get private member calibrationStatus
    void setCalibrationStatus(bool value); // set value of private member calibrationStatus

    parallelMotorControl motorController;

private:
    const int SDA_Pin = 20;
    const int SCL_Pin = 21;
    const int addr = 0x8;
    bool startProgram = false;
    bool calibrationStatus = false;
};
