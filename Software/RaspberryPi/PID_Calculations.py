import config
import time
from loggingModule import logger as lg

def getTime():
    global previousT
    currentTime = time.time()
    deltaT = currentTime - config.previousT
    config.previousT = currentTime

    return deltaT

def PID(current_x, current_y, height, width):
    x_desired = width / 2
    y_desired = height / 2
    error_x = current_x - x_desired
    error_y = current_y - y_desired
    deltaT = getTime()

    integral_x = max(min((config.integral_x + error_x*deltaT), config.integral_max), config.integral_min)
    integral_y =  max(min((config.integral_y + error_y*deltaT), config.integral_max), config.integral_min)

    derivative_x = (error_x - config.errorPrev_x)/deltaT
    derivative_y = (error_y - config.errorPrev_y)/deltaT
    config.errorPrev_x = error_x
    config.errorPrev_y = error_y

    pitch = max(min((config.Kp*error_x + config.Kd*derivative_x + config.Ki*integral_x), config.max_rotation_limit), config.min_rotation_limit)
    roll =  max(min((config.Kp*error_y + config.Kd*derivative_y + config.Ki*integral_y), config.max_rotation_limit), config.min_rotation_limit)

    return pitch, roll

if __name__ == '__main__':
    pitch, roll = PID(300, 300, 480, 480)
    lg.info("PID Calculations. Pitch: %f, Roll: %f", pitch, roll)