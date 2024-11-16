import RPI_interface
import inverseKinematics
import signal
from smbus2 import SMBus
import os
from MPU6050 import IMU

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
    
    while True:
        angle1, angle2, angle3 = map(int, input("Enter desired angles of the stepper motors: ").split())
        RPI_interface.writeInverseKinematics([angle1, angle2, angle3])
