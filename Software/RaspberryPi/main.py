import cv2 as cv
import numpy as np
import signal
import time

from inverseKinematics import encapsulatedFunction
from RPI_interface import writeInverseKinematics, handle_sigterm, handle_sigint
from PID_Calculations import PID
from loggingModule import logger

# import threading
# from queue import Queue

# frame_queue = Queue()
# pid_queue = Queue()

# def capture_and_process():
#     cap = cv.VideoCapture(0)
#     cap.set(cv.CAP_PROP_FPS, 30)
#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             break
#         height, width, _ = frame.shape
#         frame = cv.resize(frame, (480, 480))
#         hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
#         mask = cv.inRange(hsv, np.array([20, 100, 100]), np.array([30, 255, 255]))
#         contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
#         if contours:
#             largest_contour = max(contours, key=cv.contourArea)
#             ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
#             if radius > 10:
#                 frame_queue.put((x, y, height, width))
#         cv.imshow('frame', frame)
#         if cv.waitKey(1) & 0xFF == ord('q'):
#             break
#     cap.release()
#     cv.destroyAllWindows()

# def pid_and_motor_control():
#     while True:
#         if not frame_queue.empty():
#             x, y, height, width = frame_queue.get()
#             pitch, roll = PID(x, y, height, width)
#             stepperAngles = encapsulatedFunction(pitch, roll)
#             writeInverseKinematics(stepperAngles)

# if __name__ == '__main__':
#     threading.Thread(target=capture_and_process).start()
#     threading.Thread(target=pid_and_motor_control).start()


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, handle_sigterm) # kill -SIGTERM <PID>
    signal.signal(signal.SIGINT, handle_sigint) # CTRL + C
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FPS, 30)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            logger.error("Failed to grab frame")
            break
        
        height, width, channels = frame.shape
        frame = cv.resize(frame, (480, 480))
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 
        ball_color_lower = np.array([20, 100, 100])
        ball_color_upper = np.array([30, 255, 255]) 

        mask = cv.inRange(hsv, ball_color_lower, ball_color_upper)
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
            if radius > 10:
                cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                print(f"Yellow ball detected at position: ({int(x)}, {int(y)})")
                pitch, roll = PID(x, y, height, width)
                stepperAngles = encapsulatedFunction(pitch, roll)
                writeInverseKinematics(stepperAngles)
            else:
                logger.error("Contour of Yellow ball not detected!")
                
        else:
            logger.error("Yellow ball not detected!")
           
        cv.imshow('frame', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()