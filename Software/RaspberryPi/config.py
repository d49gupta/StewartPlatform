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
motors = 3
radius = 9

# Servo Motors
leg_length = 5
beta = 20
flapVector = [1.8, 0, 1.575]

# Stepper Motors
legLength1 = 4.5
legLength2 = 8
phi_zero = 0

section_angle = 360 / motors
base_motors = np.empty((motors, 3))
platform_motors = np.empty((motors, 3))