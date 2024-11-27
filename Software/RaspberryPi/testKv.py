from loggingModule import logger
from inverseKinematics import encapsulatedFunction
import RPI_interface
import config
import math
import numpy as np
import time
import cv2 as cv

prev_x = 400
prev_y = 420

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
    logger.info("Velocity: %f, Direction: %f", velocity, direction)

    return velocity, direction

def calculateOrientation(x, y):
    velocity, direction = calculateVelocity(x, y)
    currentDistance = abs(math.sqrt((x - 240)**2 + (y - 240)**2))
    logger.info("Current distance from origin: %f, velocity: %f, direction: %f", currentDistance, velocity, direction)
    tilt_x = config.Kv*velocity*math.cos(math.radians(direction))*(currentDistance/240)
    tilt_y = config.Kv*velocity*math.sin(math.radians(direction))*(currentDistance/240)
    logger.info("Kv TiltX: %f, Kv TiltY: %f", tilt_x, tilt_y)

    pitch = max(min((-tilt_x), config.max_rotation_limit), config.min_rotation_limit)
    roll =  max(min((-tilt_y), config.max_rotation_limit), config.min_rotation_limit)

    return pitch, roll

if __name__ == '__main__':
    cap = cv.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            logger.error("Failed to grab frame")
            continue

        frame = cv.resize(frame, (config.image_height, config.image_width))
        cv.circle(frame, (240, 240), 8, (0, 255, 0), -1)  # -1 means the circle is filled
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        ball_color_lower = np.array([5, 100, 100])  # Lower bound for orange
        ball_color_upper = np.array([100, 255, 255])  # Upper bound for orange

        mask = cv.inRange(hsv, ball_color_lower, ball_color_upper)
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        if contours:
                largest_contour = max(contours, key=cv.contourArea)
                ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
                if radius > 10:
                    cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    logger.info(f"Yellow ball detected at position: ({int(x)}, {int(y)})")
                    print(f"Yellow ball detected at position: ({int(x)}, {int(y)})")
                    cv.arrowedLine(frame, (int(prev_x), int(prev_y)), (int(x), int(y)), (0, 255, 0), 2, tipLength = 0.3)
                    pitch, roll = calculateOrientation(x, y)
                    stepperAngles = encapsulatedFunction(pitch, roll)
                    # RPI_interface.writeInverseKinematics(stepperAngles)
                    prev_x = x
                    prev_y = y
                else:
                    logger.error("Contour of Yellow ball not detected!")
        else:
            logger.error("Yellow ball not detected!")
        
        cv.imshow('frame', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv.destroyAllWindows()

