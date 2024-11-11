from smbus2 import SMBus
import config
import signal
from loggingModule import logger as lg

bus = SMBus(1)
inverseKinematicsCommand = 0
ESTOPCommand = 1

def handle_sigterm(signum, frame):
    lg.fatal("Graceful termination request received, shutting down process")
    writeInverseKinematics([0, 0, 0])
    exit(0)

def handle_sigint(signum, frame):
    lg.fatal("E-STOP received, shutting down process")
    writeESTOP()
    exit(0)

def writeInverseKinematics(inverseKinematics):
    data = [inverseKinematics[0], inverseKinematics[1], inverseKinematics[2]]
    
    try:
        bus.write_i2c_block_data(config.addr, inverseKinematicsCommand, data)
        lg.info("Data written successfully.")
    except IOError:
        lg.fatal("Failed to write data to the I2C peripheral")
    except Exception as e:
        lg.error(f"An error occurred: {e}")
    return True

def writeESTOP():
    try:
        bus.write_i2c_block_data(config.addr, ESTOPCommand, [])
        lg.info("Data written successfully.")
    except IOError:
        lg.fatal("Failed to write data to the I2C peripheral")
    except Exception as e:
        lg.error(f"An error occurred: {e}")
    return True

def requestData():
    try:
        data = bus.read_byte(config.addr)
        lg.info("Received data:", data)
    except IOError:
        lg.fatal("Error: Failed to read from Arduino")

if __name__ == '__main__':
    signal.signal(signal.SIGTERM, handle_sigterm) # kill -SIGTERM <PID>
    signal.signal(signal.SIGINT, handle_sigint) # CTRL + C
    while True:
        angle1, angle2, angle3 = map(int, input("Enter desired angles of the stepper motors: ").split())
        writeInverseKinematics([angle1, angle2, angle3])
