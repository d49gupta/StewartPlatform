from smbus2 import SMBus
import config
from loggingModule import logger as lg
import sys
import threading

inverseKinematicsCommand = 0
ESTOPCommand = 1
calibrationCommand = 2
bus = SMBus(1)
communicationMutex = threading.Lock()

def handle_sigterm(signum, frame):
    lg.fatal("Graceful termination request received, shutting down process")
    writeInverseKinematics([0, 0, 0])
    sys.exit(1)

def handle_sigint(signum, frame):
    lg.fatal("E-STOP received, shutting down process")
    writeESTOP()
    sys.exit(1)

def writeInverseKinematics(inverseKinematics):
    with communicationMutex:
        data = [inverseKinematics[0], inverseKinematics[1], inverseKinematics[2]]
        try:
            bus.write_i2c_block_data(config.addr, inverseKinematicsCommand, data)
            lg.info("Inverse Kinematics Data written successfully.")
        except IOError:
            lg.fatal("Inverse Kinematics failed to write data to the I2C peripheral")
            return False
        except Exception as e:
            lg.error(f"Inverse Kinematics An error occurred: {e}")
            return False

        return True

def writeESTOP():
    try:
        with communicationMutex:
            bus.write_i2c_block_data(config.addr, ESTOPCommand, [])
        lg.info("E-STOP Data written successfully.")
    except IOError:
        lg.fatal("E-STOP failed to write data to the I2C peripheral")
        sys.exit(1)
    except Exception as e:
        lg.error(f"E-STOP error occurred: {e}")
        sys.exit(1)

    return True

def requestCalibration():
    try:
        with communicationMutex:
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
        with communicationMutex:
            data = bus.read_byte(config.addr)
            lg.info("Received data: %.2f", data)
            return data
    except IOError:
        lg.fatal("Error: Failed to read from Arduino")
        sys.exit(1)

if __name__ == '__main__':
    while True:
        angle1, angle2, angle3 = map(int, input("Enter desired angles of the stepper motors: ").split())
        writeInverseKinematics([angle1, angle2, angle3])