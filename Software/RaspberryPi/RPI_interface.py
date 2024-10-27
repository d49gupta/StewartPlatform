from smbus2 import SMBus

addr = 0x8 
bus = SMBus(1)

def writeInverseKinematics(inverseKinematics):
    data = [inverseKinematics[0], inverseKinematics[1], inverseKinematics[2], inverseKinematics[3], inverseKinematics[4], inverseKinematics[5]]
    
    try:
        bus.write_i2c_block_data(addr, 0, data)
        print("Data written successfully.")
    except IOError:
        print("Failed to write data to the I2C device.")
    except Exception as e:
        print(f"An error occurred: {e}")
    return True