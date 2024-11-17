import RPI_interface
from inverseKinematics import encapsulatedFunction
import signal
from smbus2 import SMBus
import os
from MPU6050 import IMU
from PID_Calculations import PID
import config
import time

if __name__ == '__main__':
    print(f"Process ID (PID): {os.getpid()}")
    signal.signal(signal.SIGTERM, RPI_interface.handle_sigterm) # kill -SIGTERM <PID>
    signal.signal(signal.SIGINT, RPI_interface.handle_sigint) # CTRL + C

    input("Press to begin: ")
    # RPI_interface.requestCalibration()
    # while (RPI_interface.requestData() != 1): # can't continue until limit switch stage has completed
    #     pass
    # print("Limit Switch Homing Sequence Completed!")
    # imu.startStreamingIMU()
    
    # while True:
    config.previousT = time.time()
    pitch, roll = PID(0, 242, 480, 480)
    stepperAngles = encapsulatedFunction(pitch, roll)
    RPI_interface.writeInverseKinematics(stepperAngles)


