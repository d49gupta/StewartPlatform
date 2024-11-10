from smbus2 import SMBus
import config

bus = SMBus(1)
inverseKinematicsCommand = 0
ESTOPCommand = 1

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

if __name__ == '__main__':
    while True:
        angle1, angle2, angle3 = map(int, input("Enter desired angles of the stepper motors: ").split())
        writeInverseKinematics([angle1, angle2, angle3])