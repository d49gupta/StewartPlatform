import numpy as np
import math
import matplotlib.pyplot as plt
import config
from loggingModule import logger as lg

for i in range(config.motors):
    x_pos = math.cos(math.radians(i * config.section_angle)) # TODO: Change from circle to base orientation
    y_pos = math.sin(math.radians(i * config.section_angle))
    config.base_motors[i] = [config.base_radius*x_pos, config.base_radius*y_pos, 0]
    config.platform_motors[i] = [config.platform_radius*x_pos, config.platform_radius*y_pos, 0]

def input_parameters(pitch, roll):
    # Tx, Ty, Tz = map(float, input("Enter desired coordinates (Tx Ty Tz): ").split())
    coordinates = np.array([0, 0, config.starting_height])

    # pitch, roll = map(float, input("Enter pitch and roll: ").split())
    theta = math.radians(pitch)
    phi = math.radians(roll)

    rotation_matrix = np.array([
        [math.cos(theta), math.sin(theta) * math.sin(phi), math.cos(phi) * math.sin(phi)],
        [0, math.cos(phi), -math.sin(phi)],
        [-math.sin(theta), math.cos(theta) * math.sin(phi), math.cos(theta) * math.cos(phi)]
    ])
    lg.info("Coords and Rotation Matrix returned")
    return coordinates, rotation_matrix

def calculate_leg_vectors(base_motors, platform_motors, coordinates, rotation_matrix):
    leg_vectors = np.empty((config.motors, 3))
    transformed_points = np.empty((config.motors, 3))
    
    for i in range(config.motors):
        rotated_point = np.dot(rotation_matrix, platform_motors[i])
        transformed_point = rotated_point + coordinates
        transformed_points[i] = transformed_point
        leg_vectors[i] = transformed_point - base_motors[i]
    lg.info("Leg vectors and transformed points returned")
    return leg_vectors, transformed_points

def generate_circle(radius, z, num_points=100):
    angles = np.linspace(0, 2 * np.pi, num_points)
    x = radius * np.cos(angles)
    y = radius * np.sin(angles)
    z = np.full_like(x, z)
    return x, y, z

def plot_stewart_platform(base_motors, transformed_points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(base_motors[:, 0], base_motors[:, 1], base_motors[:, 2], c='b', label='Base Points')
    ax.scatter(transformed_points[:, 0], transformed_points[:, 1], transformed_points[:, 2], c='r', label='Platform Points')
    
    for i in range(config.motors):
        ax.plot([base_motors[i, 0], transformed_points[i, 0]],
                [base_motors[i, 1], transformed_points[i, 1]],
                [base_motors[i, 2], transformed_points[i, 2]], 'g-')
    
    base_circle_x, base_circle_y, base_circle_z = generate_circle(config.base_radius, 0)
    ax.plot(base_circle_x, base_circle_y, base_circle_z, 'b--', label='Base Circle')
    
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.legend()
    plt.show()
    lg.info("Stewart Platform Simulation Plotted")

def calculateVectorMagnitude(arr, power):
    x = arr[0] ** 2
    y = arr[1] ** 2
    z = arr[2] ** 2
    lg.info("Vector Magnitude Calculated")
    return (math.sqrt(x + y + z)) ** power

def calculate_alpha(e_k, f_k, g_k):
    sqrt_term = math.sqrt(e_k**2 + f_k**2)

    if -1 <= (g_k / sqrt_term) <= 1:
        sin_term = math.asin(g_k / sqrt_term)
        atan2_term = math.atan2(f_k, e_k)
        alpha_k = sin_term - atan2_term
        lg.info("Alpha Calculated Successfully. Alpha = %f", alpha_k)
        return alpha_k
    else:
        lg.error("Unable to Calculate Alpha")
        return -1


def calculateServoAngles(servo_vectors, flapVector, beta):
    servoAngleList = []
    flapMagnitude = calculateVectorMagnitude(flapVector, 1)
    for servo in servo_vectors:
        Ek = 2*flapMagnitude*servo[2]
        Fk = 2*flapMagnitude*((math.cos(math.radians(beta)))*servo[0] + math.sin(math.radians(beta))*servo[1])
        Gk = calculateVectorMagnitude(servo, 2) - (config.leg_length**2 - calculateVectorMagnitude(flapVector, 2))
        servoAngle = calculate_alpha(Ek, Fk, Gk)
        if servoAngle == -1 or -90 > servoAngle > 90: # Angular constraints of servos
            lg.error("Servo Position not Possible")
            return []
        else: 
            lg.debug("Servo Position Calculated as %f", servoAngle)
            servoAngleList.append(servoAngle)
    
    servoAnglesDegrees = [math.degrees(angle) for angle in servoAngleList]
    lg.info("Servo Angle Returned: %f", servoAngle)
    return servoAnglesDegrees

def calculateStepperAngles(stepper_vectors):
    stepperAngles = []
    for stepper in stepper_vectors:
        stepperNorm = calculateVectorMagnitude(stepper, 1)
        try:
            angle_value = (stepperNorm**2 + config.legLength1**2 - config.legLength2**2) / (2 * config.legLength1 * stepperNorm)
            lg.info("Angle Value Calculated: %f", angle_value)
            if -1 <= angle_value <= 1:
                stepperAngle = 90 + config.phi_zero - math.degrees(math.acos(angle_value))
                if stepperAngle >= 5 and stepperAngle < 0:
                    lg.warning("Negative Stepper Angle -5 < a < 0 Calculated %f", stepperAngle)
                    stepperAngle = 0
                elif(stepperAngle < -5):
                    stepperAngle = 0
                    lg.fatal("Negative Stepper Angle < -5 Calculated: %f", stepperAngle)
                elif stepperAngle >= 100:
                    stepperAngle = 100
                stepperAngles.append(int(stepperAngle))
                lg.info("Resulting Stepper Angle Calculated: %f", stepperAngle)
                
            else:
                lg.error("Position not achievable")
                # return []
        except ValueError:
            lg.error("Position not achievable due to math domain error")
            return []

    return stepperAngles

def encapsulatedFunction(pitch, roll):
    coordinates, rotation_matrix = input_parameters(pitch, roll)
    leg_vectors, _ = calculate_leg_vectors(config.base_motors, config.platform_motors, coordinates, rotation_matrix)
    stepperAngles = calculateStepperAngles(leg_vectors)
    
    return stepperAngles

if __name__ == '__main__':
    while True:
        pitch, roll = map(int, input("Enter desired pitch and roll: ").split())
        coordinates, rotation_matrix = input_parameters(pitch, roll)
        leg_vectors, transformed_points = calculate_leg_vectors(config.base_motors, config.platform_motors, coordinates, rotation_matrix)
        stepperAngles = calculateStepperAngles(leg_vectors)
        print(stepperAngles)