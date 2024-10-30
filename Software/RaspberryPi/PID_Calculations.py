import numpy as np
import math
import matplotlib.pyplot as plt
import time

x_desired = 0
y_desired = 0
Kp = 1.0
Kd = 0.025
Ki = 0.5

integral_x = 0.0
integral_y = 0.0
errorPrev_x = 0
errorPrev_y = 0
previousT = 0

integral_max = 100
integral_min = -100
max_limit = 25
min_limit = -25

def getTime():
    currentTime = time.time()
    deltaT = currentTime - previousT
    previousT = currentTime

    return deltaT

def PID(current_x, current_y):
    error_x = current_x - x_desired
    error_y = current_y - y_desired
    deltaT = getTime()

    integral_x = max(min((integral_x + error_x*deltaT), integral_max), integral_min)
    integral_y =  max(min((integral_y + error_y*deltaT), integral_max), integral_min)

    derivative_x = (error_x - errorPrev_x)/deltaT
    derivative_y = (error_y - errorPrev_y)/deltaT
    errorPrev_x = error_x
    errorPrev_y = error_y

    pitch = max(min((Kp*error_x + Kd*derivative_x + Ki*integral_x), max_limit), min_limit)
    roll =  max(min((Kp*error_y + Kd*derivative_y + Ki*integral_y), max_limit), min_limit)

    return pitch, roll