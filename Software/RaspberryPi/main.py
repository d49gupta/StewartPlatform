import cv2 as cv
import numpy as np
import signal
import threading
import config
import time
import math

from inverseKinematics import encapsulatedFunction
from RPI_interface import writeInverseKinematics, handle_sigterm, handle_sigint
from PID_Calculations import PID, getTime
from loggingModule import logger
from DataCache import CircularBuffer

class ballTracking:
    bufferSize = 100

    def __init__(self):
        self.cap = cv.VideoCapture(0)
        self.cap.set(cv.CAP_PROP_FPS, 30)

        self.positionCache = CircularBuffer(self.bufferSize)
        self.orientationCache = CircularBuffer(self.bufferSize)
        self.positionMutex = threading.Lock()
        self.orientationMutex = threading.Lock()
        self.stop_flag = threading.Event()

        self.oldInverseKinematics = [0, 0, 0]
        
        self.oldInverseKinematics = [0, 0, 0]
        self.prev_x = 0
        self.prev_y = 0
        self.velocity = 0
        self.direction = 0

    def velocityDetection(self, x, y):
        dx = x - self.prev_x
        dy = y - self.prev_y
        distance = np.sqrt(dx**2 + dy**2)
        self.velocity = distance / getTime()
        self.direction = (math.degrees(math.atan2(dy, dx)) + 360) % 360

    def positionDetection(self):
        while not self.stop_flag.is_set():
            ret, frame = self.cap.read()
            if not ret:
                logger.error("Failed to grab frame")
                continue

            frame = cv.resize(frame, (config.image_height, config.image_width))
            cv.circle(frame, (240, 240), 8, (0, 255, 0), -1)  # -1 means the circle is filled
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 
            ball_color_lower = np.array([5, 100, 100])  # Lower bound for orange
            ball_color_upper = np.array([25, 255, 255])  # Upper bound for orange


            mask = cv.inRange(hsv, ball_color_lower, ball_color_upper)
            contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            if contours:
                with self.positionMutex:
                    largest_contour = max(contours, key=cv.contourArea)
                    ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
                    if radius > 10:
                        cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                        print(x, y)
                        logger.info(f"Yellow ball detected at position: ({int(x)}, {int(y)})")
                        print(f"Yellow ball detected at position: ({int(x)}, {int(y)})")
                        # self.velocityDetection(x, y)
                        cv.arrowedLine(frame, (int(self.prev_x), int(self.prev_y)), (int(x), int(y)), (0, 255, 0), 2, tipLength = 0.3)
                        self.prev_x = x
                        self.prev_y = y
                        logger.info(f"Yellow ball detected with velocity and direction: ({self.velocity}, {self.direction})")
                        position = (int(x), int(y))
                        self.positionCache.enqueue(position)
                    else:
                        logger.error("Contour of Yellow ball not detected!")
            else:
                logger.error("Yellow ball not detected!")
            
            cv.imshow('frame', frame)
            if cv.waitKey(1) & 0xFF == ord('q'):
                self.stop()

    def calculatePID(self):
        while not self.stop_flag.is_set():
            with self.positionMutex:
                position = self.positionCache.newestValue()
                if position is None:
                    continue

            pitch, roll = PID(position[0], position[1])
            with self.orientationMutex:
                self.orientationCache.enqueue((pitch, roll))

    def calculateAngles(self):
        while not self.stop_flag.is_set():
            with self.orientationMutex:
                orientation = self.orientationCache.newestValue()
                if orientation is None:
                    continue
                    
            stepperAngles = encapsulatedFunction(orientation[0], orientation[1])
            if any(abs(new - old) > config.angle_threshold
                for new, old in zip(stepperAngles, self.oldInverseKinematics)):
                if writeInverseKinematics(stepperAngles):
                    self.oldInverseKinematics = stepperAngles
            else:
                logger.warning("Angles not sent, too similar of values")

    def stop(self):
        self.cap.release()
        cv.destroyAllWindows()
        self.stop_flag.set()

if __name__ == '__main__':
    signal.signal(signal.SIGTERM, handle_sigterm) # kill -SIGTERM <PID>
    signal.signal(signal.SIGINT, handle_sigint) # CTRL + C

    ball = ballTracking()
    input("Press to begin: ")
    threading.Thread(target=ball.positionDetection).start()
    time.sleep(0.01)
    threading.Thread(target=ball.calculatePID).start()
    time.sleep(0.01)
    threading.Thread(target=ball.calculateAngles).start()