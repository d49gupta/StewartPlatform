import cv2 as cv
import numpy as np
from time import sleep

def detect_yellow_ball():
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FPS, 30)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
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
                cv.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)
                print(f"Yellow ball detected at position: ({int(x)}, {int(y)})")

        cv.imshow('frame', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()

if __name__ == '__main__':
    detect_yellow_ball()
