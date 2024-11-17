import cv2 as cv
import numpy as np
import signal
import time

from inverseKinematics import encapsulatedFunction
from RPI_interface import writeInverseKinematics, handle_sigterm, handle_sigint
from PID_Calculations import PID
from loggingModule import logger

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
        ball_color_lower = np.array([10, 100, 100])
        ball_color_upper = np.array([25, 255, 255]) 

        mask = cv.inRange(hsv, ball_color_lower, ball_color_upper)
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
            if radius > 10:
                cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                print(f"Yellow ball detected at position: ({int(x)}, {int(y)})")
                pitch, roll = PID(x, y, height, width)
                print(f"Pitch and roll calculated: ({int(pitch)}, {int(roll)})")
                stepperAngles = encapsulatedFunction(pitch, roll)
                print(f"Stepper angles calculated: ({int(stepperAngles[0])}, {int(stepperAngles[1])}, {int(stepperAngles[2])})")
                writeInverseKinematics(stepperAngles) # TODO: Add acknowledgement from Arduino
                time.sleep(0.1)
                print("angles sent")
            else:
                logger.error("Contour of Yellow ball not detected!")
                
        else:
            logger.error("Yellow ball not detected!")
           

        cv.imshow('frame', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()