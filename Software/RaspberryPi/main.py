import cv2 as cv
import numpy as np
import signal
import threading
import config
from DataCache import CircularBuffer

from inverseKinematics import encapsulatedFunction
from RPI_interface import writeInverseKinematics, handle_sigterm, handle_sigint
from PID_Calculations import PID
from loggingModule import logger

class ballTracking:
    bufferSize = 10

    def __init__(self):
        self.cap = cv.VideoCapture(0)
        self.cap.set(cv.CAP_PROP_FPS, 30)

        self.positionCache = CircularBuffer(self.bufferSize)
        self.orientationCache = CircularBuffer(self.bufferSize)
        self.positionMutex = threading.Lock()
        self.orientationMutex = threading.Lock()
        self.stop_flag = threading.Event()

    def positionDetection(self):
        while not self.stop_flag.is_set():
            ret, frame = self.cap.read()
            if not ret:
                logger.error("Failed to grab frame")
                break
            
            frame = cv.resize(frame, (config.image_height, config.image_width))
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 
            ball_color_lower = np.array([20, 100, 100])
            ball_color_upper = np.array([30, 255, 255]) 

            mask = cv.inRange(hsv, ball_color_lower, ball_color_upper)
            contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            if contours:
                with self.positionMutex:
                    largest_contour = max(contours, key=cv.contourArea)
                    ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
                    if radius > 10:
                        cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                        logger.info(f"Yellow ball detected at position: ({int(x)}, {int(y)})")
                        print(f"Yellow ball detected at position: ({int(x)}, {int(y)})")
                        position = (int(x), int(y))
                        self.positionCache.enqueue(position)
                    else:
                        logger.error("Contour of Yellow ball not detected!")
            else:
                logger.error("Yellow ball not detected!")
            
            # cv.imshow('frame', frame)
            if cv.waitKey(1) & 0xFF == ord('q'):
                self.stop()

    def calculatePID(self):
        while not self.stop_flag.is_set():
            with self.positionMutex:
                position = self.positionCache.newestValue()
            
            pitch, roll = PID(position[0], position[1])
            
            with self.orientationMutex:
                self.orientationCache.enqueue((pitch, roll))

    def calculateAngles(self):
        while not self.stop_flag.is_set():
            with self.orientationMutex:
                orientation = self.orientationCache.newestValue()
                stepperAngles = encapsulatedFunction(orientation[0], orientation[1])
                if not writeInverseKinematics(stepperAngles):
                    self.stop()

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
    threading.Thread(target=ball.calculatePID).start()
    threading.Thread(target=ball.calculateAngles).start()