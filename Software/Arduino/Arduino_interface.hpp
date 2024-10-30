#include <Wire.h>
#include <ArduinoSTL.h>
#include <vector>

class ArduinoInterface {
public: 
    std::vector<int> inverseKinematics;
    const int SDA_Pin = 20;
    const int SCL_Pin = 21;
    const int addr = 0x8;
    const int baudRate = 9600;

    void receiveI2C(int data);
};
