#include <Wire.h>
#include <ArduinoSTL.h>
#include <vector>

class ArduinoInterface {
public: 
    const int baudRate = 9600;
    std::vector<int> inverseKinematics = {0, 0, 0, 0};
    void setup();
    void receiveI2C();
    voind sendI2C(int data);

private:
    const int SDA_Pin = 20;
    const int SCL_Pin = 21;
    const int addr = 0x8;
};
