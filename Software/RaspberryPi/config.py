import numpy as np

Kp = 0.075
Kd = 0.025
Ki = 0.05

integral_x = 0.0
integral_y = 0.0
errorPrev_x = 0
errorPrev_y = 0
previousT = 0

integral_max = 100
integral_min = -100
max_rotation_limit = 20
min_rotation_limit = -20

addr = 0x8
imu_addr = 0x68 
motors = 3
base_radius = 8
platform_radius = 12
starting_height = 4.12 # starting height for 0 degrees for each motor

# Servo Motors
leg_length = 5
beta = 20
flapVector = [1.8, 0, 1.575]

# Stepper Motors
legLength1 = 4
legLength2 = 7
phi_zero = 0

section_angle = 360 / motors
base_motors = np.empty((motors, 3))
platform_motors = np.empty((motors, 3))