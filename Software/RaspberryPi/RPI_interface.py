from smbus2 import SMBus
import config
import signal

bus = SMBus(1)
inverseKinematicsCommand = 0
ESTOPCommand = 1

def handle_sigterm(signum, frame):
    print("E-STOP Received, shutting down process")
    writeESTOP()
    exit(0)

def writeInverseKinematics(inverseKinematics):
    data = [inverseKinematics[0], inverseKinematics[1], inverseKinematics[2]]
    
    try:
        bus.write_i2c_block_data(config.addr, inverseKinematicsCommand, data)
        print("Data written successfully.")
    except IOError:
        print("Failed to write data to the I2C peripheral")
    except Exception as e:
        print(f"An error occurred: {e}")
    return True

def writeESTOP():
    try:
        bus.write_i2c_block_data(config.addr, ESTOPCommand, [])
        print("Data written successfully.")
    except IOError:
        print("Failed to write data to the I2C peripheral")
    except Exception as e:
        print(f"An error occurred: {e}")
    return True

def requestData():
    try:
        data = bus.read_byte(config.addr)
        print("Received data:", data)
    except IOError:
        print("Error: Failed to read from Arduino")

if __name__ == '__main__':
    signal.signal(signal.SIGTERM, handle_sigterm) # kill -SIGTERM <PID>
    while True:
        angle1, angle2, angle3 = map(int, input("Enter desired angles of the stepper motors: ").split())
        writeInverseKinematics([angle1, angle2, angle3])
    requestData()