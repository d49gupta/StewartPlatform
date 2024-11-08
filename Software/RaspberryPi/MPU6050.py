from smbus2 import SMBus
from PID_Calculations import getTime
import math

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

acc_x, acc_y, acc_z = 0, 0, 0
gyro_x, gyro_y, gyro_z = 0, 0, 0
accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ = 0, 0, 0, 0, 0
roll, pitch, yaw = 0, 0, 0
AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroError = 0, 0, 0, 0, 0
counter = 0

def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)

    #concatenate higher and lower value
    value = ((high << 8) | low)
    
    #to get signed value from mpu6050
    if(value > 32768):
            value = value - 65536
    return value

def calculateOrientation(acc_x, acc_y, gyro_x, gyro_y, acc_z, gyro_z):
    accAngleX = (math.atan(acc_y / math.sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 180 / math.pi) - 0.58
    accAngleY = (math.atan(-1 * acc_x / math.sqrt(pow(acc_y, 2) + pow(acc_z, 2))) * 180 / math.pi) + 1.58
    gyro_x = gyro_x + 0.56
    gyro_y = gyro_y - 2
    gyro_z = gyro_z + 0.79

    elapsedTime = getTime()
    gyroAngleX = gyroAngleX + gyro_x * elapsedTime 
    gyroAngleY = gyroAngleY + gyro_y * elapsedTime
    yaw =  yaw + gyro_z * elapsedTime
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY

    return pitch, roll, yaw

# def calibration():
      
if __name__ == '__main__':
    bus = SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
    Device_Address = 0x68   # MPU6050 device address

    MPU_Init()
    print (" Reading Data of Gyroscope and Accelerometer")

    while True:
        #Read Accelerometer raw value
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)
        
        #Read Gyroscope raw value
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)
        
        #Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0
        
        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0
        
        print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
