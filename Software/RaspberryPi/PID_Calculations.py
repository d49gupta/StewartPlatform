import numpy as np
import config
import matplotlib.pyplot as plt
import time

def getTime():
    currentTime = time.time()
    deltaT = currentTime - previousT
    previousT = currentTime

    return deltaT

def PID(current_x, current_y, height, width):
    x_desired = width / 2
    y_desired = height / 2
    error_x = current_x - x_desired
    error_y = current_y - y_desired
    deltaT = getTime()

    integral_x = max(min((integral_x + error_x*deltaT), config.integral_max), config.integral_min)
    integral_y =  max(min((integral_y + error_y*deltaT), config.integral_max), config.integral_min)

    derivative_x = (error_x - errorPrev_x)/deltaT
    derivative_y = (error_y - errorPrev_y)/deltaT
    errorPrev_x = error_x
    errorPrev_y = error_y

    pitch = max(min((config.Kp*error_x + config.Kd*derivative_x + config.Ki*integral_x), config.max_rotation_limit), config.min_rotation_limit)
    roll =  max(min((config.Kp*error_y + config.Kd*derivative_y + config.Ki*integral_y), config.max_rotation_limit), config.min_rotation_limit)

    return pitch, roll