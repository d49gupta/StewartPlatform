from smbus2 import SMBus
from PID_Calculations import getTime
import math
import config

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

gyroAngleX, gyroAngleY, gyroAngleZ, yaw = 0, 0, 0, 0
AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ = 0, 0, 0, 0, 0

def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(config.imu_addr, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(config.imu_addr, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(config.imu_addr, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(config.imu_addr, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(config.imu_addr, INT_ENABLE, 1)

def read_raw_data(addr, gyro = True):
	#Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(config.imu_addr, addr)
    low = bus.read_byte_data(config.imu_addr, addr+1)

    #concatenate higher and lower value
    value = ((high << 8) | low)
    
    #to get signed value from mpu6050
    if(value > 32768):
            value = value - 65536
    
    if gyro:
         return value / 131.0
    else:
         return value / 16384.0

def calculateOrientation(acc_x, acc_y, gyro_x, gyro_y, acc_z, gyro_z):
    global gyroAngleX, gyroAngleY, gyroAngleZ, yaw
    accAngleX = (math.atan(acc_y / math.sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 180 / math.pi) - AccErrorX # choose to keep AccErrorX/AccErrorY only for increased accuracy, if too slow get rid of calculation
    accAngleY = (math.atan(acc_x / math.sqrt(pow(acc_y, 2) + pow(acc_z, 2))) * 180 / math.pi) + AccErrorY
    gyro_x = gyro_x + GyroErrorX
    gyro_y = gyro_y - GyroErrorY
    gyro_z = gyro_z + GyroErrorZ

    elapsedTime = getTime()
    gyroAngleX = gyroAngleX + gyro_x * elapsedTime 
    gyroAngleY = gyroAngleY + gyro_y * elapsedTime
    yaw =  yaw + gyro_z * elapsedTime
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY

    return pitch, roll, yaw

def calibration():
    global AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ
    counter = 0
    while (counter < 200):
        acc_x = read_raw_data(ACCEL_XOUT_H, False)
        acc_y = read_raw_data(ACCEL_YOUT_H, False)
        acc_z = read_raw_data(ACCEL_ZOUT_H, False)
        gyro_x = read_raw_data(GYRO_XOUT_H, True)
        gyro_y = read_raw_data(GYRO_YOUT_H, True)
        gyro_z = read_raw_data(GYRO_ZOUT_H, True)

        AccErrorX = AccErrorX + ((math.atan((acc_y) / math.sqrt(pow((acc_x), 2) + pow((acc_z), 2))) * 180 / math.pi))
        AccErrorY = AccErrorY + ((math.atan(-1 * (acc_x) / math.sqrt(pow((acc_y), 2) + pow((acc_z), 2))) * 180 / math.pi))
        GyroErrorX = GyroErrorX + (gyro_x)
        GyroErrorY = GyroErrorY + (gyro_y)
        GyroErrorZ = GyroErrorZ + (gyro_z)
        counter += 1

    AccErrorX = AccErrorX / 200
    AccErrorY = AccErrorY / 200
    GyroErrorX - GyroErrorX / 200
    GyroErrorY - GyroErrorY / 200
    GyroErrorZ - GyroErrorZ / 200

def simpleCalculation(acc_x, acc_y, acc_z):
    roll = math.atan2(acc_y, acc_z)*180/math.pi
    pitch = ((math.atan(acc_x / math.sqrt(pow((acc_y), 2) + pow((acc_z), 2))) * 180 / math.pi))

    return roll, pitch
      
if __name__ == '__main__':
    bus = SMBus(1)

    MPU_Init()
    print("Calibrating IMU")
    calibration()
    print (" Reading Data of Gyroscope and Accelerometer")

    while True:
        acc_x = read_raw_data(ACCEL_XOUT_H, False)
        acc_y = read_raw_data(ACCEL_YOUT_H, False)
        acc_z = read_raw_data(ACCEL_ZOUT_H, False)
        gyro_x = read_raw_data(GYRO_XOUT_H, True)
        gyro_y = read_raw_data(GYRO_YOUT_H, True)
        gyro_z = read_raw_data(GYRO_ZOUT_H, True)
        
        print ("Gx=%.2f" %gyro_x, u'\u00b0'+ "/s", "\tGy=%.2f" %gyro_y, u'\u00b0'+ "/s", "\tGz=%.2f" %gyro_z, u'\u00b0'+ "/s", "\tAx=%.2f g" %acc_x, "\tAy=%.2f g" %acc_y, "\tAz=%.2f g" %acc_z)
        pitch, roll, yaw = calculateOrientation(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
        print ("Pitch=%.2f" %pitch, u'\u00b0', "\tRoll=%.2f" %roll, u'\u00b0', "\tYaw=%.2f" %yaw, u'\u00b0')
        roll1, pitch1 = simpleCalculation(acc_x, acc_y, acc_z)
        print ("Pitch=%.2f" %pitch1, u'\u00b0', "\tRoll=%.2f" %roll1, u'\u00b0')