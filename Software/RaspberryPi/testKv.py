
import config
import math
import numpy as np
import time

prev_x = 240
prev_y = 240

def getTime():
    global previousT
    currentTime = time.time()
    deltaT = currentTime - config.previousT
    config.previousT = currentTime

    return deltaT

def calculateVelocity(x, y):
    dx = x - prev_x
    dy = y - prev_y
    distance = np.sqrt(dx**2 + dy**2)
    velocity = distance / getTime()
    direction = (math.degrees(math.atan2(dy, dx)) + 360) % 360 # 180 is left of image, 90 is top of image, 270 is bottom and 0 is right of image

    return velocity, direction

def calculateOrientation(x, y):
    velocity, direction = calculateVelocity(x, y)
    print(velocity, direction)
    currentDistance = abs(math.sqrt((x - 240)**2 + (y - 240)**2))
    tilt_x = config.Kv*velocity*math.cos(math.radians(direction))*(currentDistance/240)
    tilt_y = config.Kv*velocity*math.sin(math.radians(direction))*(currentDistance/240)

    return tilt_x, tilt_y

if __name__ == '__main__':
    time.sleep(0.001)
    tilt_x, tilt_y = calculateOrientation(150, 150)
    print(tilt_x, tilt_y)


