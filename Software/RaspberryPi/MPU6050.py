from smbus2 import SMBus
from PID_Calculations import getTime
import math
import config
from loggingModule import logger as lg
from DataCache import CircularBuffer
import threading
class MPU6050:
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
    bufferSize = 10

    def __init__(self):
        self.bus = SMBus(1)

        self.bus.write_byte_data(config.imu_addr, self.SMPLRT_DIV, 7) # write to sample rate register
        self.bus.write_byte_data(config.imu_addr, self.PWR_MGMT_1, 1) # write to power management register
        self.bus.write_byte_data(config.imu_addr, self.CONFIG, 0) # write to configuration register
        self.bus.write_byte_data(config.imu_addr, self.GYRO_CONFIG, 24) # write to gyro configuration register
        self.bus.write_byte_data(config.imu_addr, self.INT_ENABLE, 1) # write to interrupt enable register

        self.dataCache = CircularBuffer(self.bufferSize)
        self.mutex = threading.lock()
        self.stop_flag = threading.Event() # flag to stop threads if needed

    def read_raw_data(self, addr, gyro = True):
        #Accelero and Gyro value are 16-bit
        high = self.bus.read_byte_data(config.imu_addr, addr)
        low = self.bus.read_byte_data(config.imu_addr, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
                lg.debug("Signed Value Adjusted to %f", value)
        
        if gyro:
            return value / 131.0
        else:
            return value / 16384.0

    def calculateOrientation(self, acc_x, acc_y, gyro_x, gyro_y, acc_z, gyro_z):
        global gyroAngleX, gyroAngleY, gyroAngleZ, yaw
        accAngleX = (math.atan(acc_y / math.sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 180 / math.pi) - self.AccErrorX # choose to keep AccErrorX/AccErrorY only for increased accuracy, if too slow get rid of calculation
        accAngleY = (math.atan(acc_x / math.sqrt(pow(acc_y, 2) + pow(acc_z, 2))) * 180 / math.pi) + self.AccErrorY
        gyro_x = gyro_x + self.GyroErrorX
        gyro_y = gyro_y - self.GyroErrorY
        gyro_z = gyro_z + self.GyroErrorZ

        elapsedTime = getTime()
        gyroAngleX = gyroAngleX + gyro_x * elapsedTime 
        gyroAngleY = gyroAngleY + gyro_y * elapsedTime
        yaw =  yaw + gyro_z * elapsedTime
        roll = 0.96 * gyroAngleX + 0.04 * accAngleX
        pitch = 0.96 * gyroAngleY + 0.04 * accAngleY

        lg.info("Pitch=%.2f%s\tRoll=%.2f%s\tYaw=%.2f%s" % (pitch, u'\u00b0', roll, u'\u00b0', yaw, u'\u00b0'))
        return pitch, roll, yaw

    def calibration(self):
        counter = 0
        while (counter < 200):
            acc_x = self.read_raw_data(self.ACCEL_XOUT_H, False)
            acc_y = self.read_raw_data(self.ACCEL_YOUT_H, False)
            acc_z = self.read_raw_data(self.ACCEL_ZOUT_H, False)
            gyro_x = self.read_raw_data(self.GYRO_XOUT_H, True)
            gyro_y = self.read_raw_data(self.GYRO_YOUT_H, True)
            gyro_z = self.read_raw_data(self.GYRO_ZOUT_H, True)

            lg.debug("Gx=%.2f%s/s\tGy=%.2f%s/s\tGz=%.2f%s/s\tAx=%.2f g\tAy=%.2f g\tAz=%.2f g" % (gyro_x, u'\u00b0', gyro_y, u'\u00b0', gyro_z, u'\u00b0', acc_x, acc_y, acc_z))

            self.AccErrorX = self.AccErrorX + ((math.atan((acc_y) / math.sqrt(pow((acc_x), 2) + pow((acc_z), 2))) * 180 / math.pi))
            self.AccErrorY = self.AccErrorY + ((math.atan(-1 * (acc_x) / math.sqrt(pow((acc_y), 2) + pow((acc_z), 2))) * 180 / math.pi))
            self.GyroErrorX = self.GyroErrorX + (gyro_x)
            self.GyroErrorY = self.GyroErrorY + (gyro_y)
            self.GyroErrorZ = self.GyroErrorZ + (gyro_z)
            counter += 1

        self.AccErrorX = self.AccErrorX / 200
        self.AccErrorY = self.AccErrorY / 200
        self.GyroErrorX = self.GyroErrorX / 200
        self.GyroErrorY = self.GyroErrorY / 200
        self.GyroErrorZ = self.GyroErrorZ / 200

    def calculatePitchRoll(self):
        acc_x = self.read_raw_data(self.ACCEL_XOUT_H, False)
        acc_y = self.read_raw_data(self.ACCEL_YOUT_H, False)
        acc_z = self.read_raw_data(self.ACCEL_ZOUT_H, False)
        roll = math.atan2(acc_y, acc_z)*180/math.pi
        pitch = ((math.atan(acc_x / math.sqrt(pow((acc_y), 2) + pow((acc_z), 2))) * 180 / math.pi))

        lg.info("Pitch=%.2f%s\tRoll=%.2f%s" % (pitch, u'\u00b0', roll, u'\u00b0'))
        return (pitch, roll)
    
    def cacheSensorReadings(self):
        while not self.stop_flag.is_set(): # keep running until stop_flag is set
            with self.mutex:
                sensorData = self.calculateOrientation()
                self.dataCache.enqueue(sensorData)

    def readCacheReadings(self):
        while not self.stop_flag.is_set(): # keep running until stop_flag is set
            with self.mutex:
                val = self.dataCache.newestValue()
                # write to arduino
                lg.info("Pitch=%.2f%s\tRoll=%.2f%s" % (val[0], u'\u00b0', val[1], u'\u00b0'))
                return val
    
    def stop(self):
        self.stop_flag.set()  # Signal both threads to stop

    def startStreamingIMU(self):
        producerThread = threading.Thread(target=self.cacheSensorReadings)
        consumerThread = threading.Thread(target=self.readCacheReadings)

        producerThread.start()
        consumerThread.start()

        producerThread.join()
        consumerThread.join()

        
if __name__ == '__main__':
    imu = MPU6050()
    