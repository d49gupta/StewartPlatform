#include <Wire.h>
#include <ArduinoSTL.h>
#include <vector>

const int SDA_Pin = 20;
const int SCL_Pin = 21;
const int addr = 0x8;
const int baudRate = 9600;

class ArduinoInterface {
public: 
    std::vector<int> inverseKinematics;
    void receiveI2C();
};
