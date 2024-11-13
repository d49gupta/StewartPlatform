from smbus2 import SMBus
import config
import signal
from loggingModule import logger as lg
import os
import sys
from MPU6050 import IMU

inverseKinematicsCommand = 0
ESTOPCommand = 1
calibrationCommand = 2

def handle_sigterm(signum, frame):
    lg.fatal("Graceful termination request received, shutting down process")
    writeInverseKinematics([0, 0, 0])
    sys.exit(0)

def handle_sigint(signum, frame):
    lg.fatal("E-STOP received, shutting down process")
    writeESTOP()
    exit(0)

def writeInverseKinematics(inverseKinematics):
    data = [inverseKinematics[0], inverseKinematics[1], inverseKinematics[2]]
    
    try:
        bus.write_i2c_block_data(config.addr, inverseKinematicsCommand, data)
        lg.info("Inverse Kinematics Data written successfully.")
    except IOError:
        lg.fatal("Inverse Kinematics failed to write data to the I2C peripheral")
        sys.exit(1)
    except Exception as e:
        lg.error(f"Inverse Kinematics An error occurred: {e}")
        sys.exit(1)

    return True

def writeESTOP():
    try:
        bus.write_i2c_block_data(config.addr, ESTOPCommand, [])
        lg.info("E-STOP Data written successfully.")
    except IOError:
        lg.fatal("E-STOP failed to write data to the I2C peripheral")
        sys,exit(1)
    except Exception as e:
        lg.error(f"E-STOP error occurred: {e}")
        sys.exit(1)

    return True

def requestCalibration():
    try:
        bus.write_i2c_block_data(config.addr, calibrationCommand, [])
        lg.info("Calibration request written successfully.")
    except IOError:
        lg.fatal("Calibration request faild to write data to the I2C peripheral")
        sys.exit(1)
    except Exception as e:
        lg.error(f"Calibration request failed with error: {e}")
        sys.exit(1)

    return True

def requestData():
    try:
        data = bus.read_byte(config.addr)
        lg.info("Received data: %.2f", data)
        return data
    except IOError:
        lg.fatal("Error: Failed to read from Arduino")
        sys.exit(1)

if __name__ == '__main__':
    bus = SMBus(1)
    imu = IMU(bus)
    print(f"Process ID (PID): {os.getpid()}")
    signal.signal(signal.SIGTERM, handle_sigterm) # kill -SIGTERM <PID>
    signal.signal(signal.SIGINT, handle_sigint) # CTRL + C

    requestCalibration()
    while (requestData() != 1): # can't continue until limit switch stage has completed
        pass
    imu.startStreamingIMU()
    print("Homing sequence completed!")
    
    while True:
        angle1, angle2, angle3 = map(int, input("Enter desired angles of the stepper motors: ").split())
        writeInverseKinematics([angle1, angle2, angle3])
