import numpy as np

Kp = 1.0
Kd = 0.025
Ki = 0.5

integral_x = 0.0
integral_y = 0.0
errorPrev_x = 0
errorPrev_y = 0
previousT = 0

integral_max = 100
integral_min = -100
max_rotation_limit = 25
min_rotation_limit = -25

addr = 0x8 
motors = 3
radius = 5

# Servo Motors
leg_length = 5
beta = 20
flapVector = [1.8, 0, 1.575]

# Stepper Motors
legLength1 = 6
legLength2 = 33
phi_zero = 25

section_angle = 360 / motors
base_motors = np.empty((motors, 3))
platform_motors = np.empty((motors, 3))