import numpy as np
import time

Kp = 0.033
Kd = 0.076
Ki = 0.15
Kv = 0.008

integral_x = 0.0
integral_y = 0.0
errorPrev_x = 0
errorPrev_y = 0
previousT = time.time()

integral_max = 1000
integral_min = -1000
max_rotation_limit = 17
min_rotation_limit = -17

addr = 0x8
imu_addr = 0x68 
motors = 3
base_radius = 8
platform_radius = 11.4
starting_height = 6.25 # starting height for 0 degrees for each motor (4.125), optimal starting height for max rotation (6.25), starting height at 0 (5.5)

# Servo Motors
leg_length = 5
beta = 20
flapVector = [1.8, 0, 1.575]

# Stepper Motors
legLength1 = 4
legLength2 = 7
phi_zero = 35 # return from homing sequence

section_angle = 360 / motors
base_motors = np.empty((motors, 3))
platform_motors = np.empty((motors, 3))

angle_threshold = 4

image_height = 480
image_width = 480
desiredPoints = [(336, 298), (135, 160)]
