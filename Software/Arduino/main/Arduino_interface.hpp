#include <Wire.h>
#include <ArduinoSTL.h>
#include <vector>

class ArduinoInterface {
public: 
    void receiveI2C();
    void setup();
    const int baudRate = 9600;
    std::vector<int> inverseKinematics = {0, 0, 0, 0};

private:
    const int SDA_Pin = 20;
    const int SCL_Pin = 21;
    const int addr = 0x8;
};
