import config
import time
from loggingModule import logger as lg
from inverseKinematics import encapsulatedFunction 
from RPI_interface import writeInverseKinematics
from testKv import calculateOrientation, calculateVelocity

def getTime():
    global previousT
    currentTime = time.time()
    deltaT = currentTime - config.previousT
    config.previousT = currentTime

    return deltaT

def PID(current_x, current_y):
    x_desired = config.image_width / 2
    y_desired = config.image_height / 2
    error_x = current_x - x_desired
    error_y = current_y - y_desired
    deltaT = getTime()
    lg.info("delta T %f ", deltaT)
    lg.info("raw error x %f ", error_x)
    lg.info("raw error y %f ", error_y)

    integral_x = config.integral_x + error_x*deltaT
    integral_y = config.integral_y + error_y*deltaT
    lg.info("Raw integral Calculations. x: %f, y: %f", integral_x, integral_y)
    
    integral_x = max(min((integral_x), config.integral_max), config.integral_min)
    integral_y =  max(min((integral_y), config.integral_max), config.integral_min)
    
    derivative_x = (error_x - config.errorPrev_x)/deltaT
    derivative_y = (error_y - config.errorPrev_y)/deltaT

    lg.info("Raw derivate Calculations. x: %f, y: %f", derivative_x, derivative_y)
    config.errorPrev_x = error_x
    config.errorPrev_y = error_y

    pitch = config.Kp*error_x + config.Kd*derivative_x + config.Ki*integral_x
    roll = config.Kp*error_y + config.Kd*derivative_y + config.Ki*integral_y

    # print(config.Kp*error_x, config.Kd*derivative_x, config.Ki*integral_x)
    # print(config.Kp*error_y, config.Kd*derivative_y, config.Ki*integral_y)
    
    lg.info("Raw PID Calculations. Pitch: %f, Roll: %f", pitch, roll)
    
    pitch = max(min((-pitch), config.max_rotation_limit), config.min_rotation_limit)
    roll =  max(min((-roll), config.max_rotation_limit), config.min_rotation_limit)

    lg.info("PID Calculations. Pitch: %f, Roll: %f", pitch, roll)
    return pitch, roll

if __name__ == '__main__':
    pitch, roll = PID(400, 420)
    velocity, direction = calculateVelocity(200, 200)
    pitch1, roll1, = calculateOrientation(200, 200)
    print(pitch, roll, pitch1, roll1)
    print(pitch + pitch1, roll + roll1)
    stepperAngles = encapsulatedFunction(pitch + pitch1, roll + roll1)
    writeInverseKinematics(stepperAngles)