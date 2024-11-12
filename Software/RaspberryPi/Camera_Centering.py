import cv2
import numpy as np
import time
from loggingModule import logger as lg

def track_red_circle(smoothing_factor=0.4, alignment_threshold=20, alignment_duration=3):
    """
    Tracks red circles in a video feed and checks for alignment with the center of the frame.
    
    Args:
        smoothing_factor (float): The factor to smooth the detected position of the circle.
        alignment_threshold (int): The distance in pixels within which the circle is considered aligned.
        alignment_duration (int): The duration in seconds for which the circle must remain aligned to confirm alignment.

    Returns:
        bool: True if alignment is confirmed.
    """
    # Initialize the webcam (0 for the default camera)
    cap = cv2.VideoCapture(0)

    # Variables to track the alignment time and smoothing
    alignment_start_time = None
    alignment_confirmed = False
    smoothed_x, smoothed_y = None, None  # To store smoothed position of the circle

    while True:
        # Capture each frame
        ret, frame = cap.read()
        if not ret:
            lg.error("Frame not grabbed")
            break

        # Get the frame dimensions
        height, width, _ = frame.shape
        center_x, center_y = width // 2, height // 2

        # Draw a marker at the center of the frame
        cv2.drawMarker(frame, (center_x, center_y), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

        # Convert the frame to HSV for color filtering
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the range for the red color in HSV
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Create masks to detect red color in both hue ranges
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        # Apply the mask to the original frame
        red_only = cv2.bitwise_and(frame, frame, mask=mask)

        # Convert the masked image to grayscale for circle detection
        gray = cv2.cvtColor(red_only, cv2.COLOR_BGR2GRAY)

        # Detect circles using Hough Circle Transform with a maximum radius constraint
        circles = cv2.HoughCircles(
            gray, 
            cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
            param1=100, param2=30, minRadius=20, maxRadius=50
        )

        if circles is not None:
            # Convert the (x, y, radius) values to integers
            circles = np.round(circles[0, :]).astype("int")
            
            # Process only the first detected circle
            x, y, r = circles[0]
            lg.debug("Circle Detected (%f, %f, %f)", x, y, r)

            # Smooth the detected position using a moving average
            if smoothed_x is None or smoothed_y is None:
                smoothed_x, smoothed_y = x, y  # Initialize smoothing with the first detected position
                lg.info("Smoothing Initialized")
            else:
                smoothed_x = int(smoothed_x * (1 - smoothing_factor) + x * smoothing_factor)
                smoothed_y = int(smoothed_y * (1 - smoothing_factor) + y * smoothing_factor)

            # Draw the smoothed circle and its center
            cv2.circle(frame, (smoothed_x, smoothed_y), r, (255, 0, 0), 2)
            cv2.circle(frame, (smoothed_x, smoothed_y), 2, (0, 0, 255), 3)

            # Check if the circle is aligned with the center marker
            if abs(smoothed_x - center_x) <= alignment_threshold and abs(smoothed_y - center_y) <= alignment_threshold:
                # Start timing if it's the first alignment
                if alignment_start_time is None:
                    alignment_start_time = time.time()
                    lg.debug("Alignment Timer Started")

                # Check if it's been aligned for at least the specified duration
                elif time.time() - alignment_start_time >= alignment_duration:
                    if not alignment_confirmed:
                        cv2.putText(frame, "Alignment confirmed!", (center_x - 100, center_y - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        lg.info("Camera Alignment Confirmed (%f)", time.time())
                        alignment_confirmed = True
                        cap.release()
                        cv2.destroyAllWindows()
                        return True  # Return True once alignment is confirmed
            else:
                # Reset the alignment timer if the circle is no longer aligned
                alignment_start_time = None
                alignment_confirmed = False
                lg.info("Alignment Interrupted at %f's", time.time())
        else:
            # Reset if no circle is detected
            alignment_start_time = None
            alignment_confirmed = False
            lg.info("No Circle Detected")

        # Display the result
        cv2.imshow("Live Video Feed", frame)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            lg.debug("Live Feed Ended")
            break

    # Release the capture and close all windows
    cap.release()
    cv2.destroyAllWindows()
    return False  # Return False if the function ends without confirming alignment

if __name__ == '__main__':
    result = track_red_circle()
    if result:
        lg.info("Alignment was successful!")
    else:
        lg.fatal("Alignment was not achieved.")
