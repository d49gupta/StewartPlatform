#include <Wire.h>
#include <ArduinoSTL.h>
#include <vector>

class ArduinoInterface {
public: 
    const int baudRate = 9600;
    static const int datatoSend = 40;
    std::vector<int> inverseKinematics = {0, 0, 0, 0};
    void setup(); // setup I2C pins and master address
    void receiveI2C(); // receive I2C data from rpi and save numbers in vector or for E-STOP
    static void sendI2C();  // send I2C data from arduino to rpi

private:
    const int SDA_Pin = 20;
    const int SCL_Pin = 21;
    const int addr = 0x8;
};
