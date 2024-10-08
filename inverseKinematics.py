import numpy as np
import math
import matplotlib.pyplot as plt

motors = 6
radius = 5
leg_length = 5
beta = 20
flapVector = [1.8, 0, 1.575]


section_angle = 360 / motors
base_motors = np.empty((motors, 3))
platform_motors = np.empty((motors, 3))

for i in range(motors):
    x = radius * math.cos(math.radians(i * section_angle))
    y = radius * math.sin(math.radians(i * section_angle))
    base_motors[i] = [x, y, 0]
    platform_motors[i] = [x, y, 0]

def input_parameters():
    Tx, Ty, Tz = map(float, input("Enter desired coordinates (Tx Ty Tz): ").split())
    coordinates = np.array([Tx, Ty, Tz])

    pitch, roll = map(float, input("Enter pitch and roll: ").split())
    theta = math.radians(pitch)
    phi = math.radians(roll)

    rotation_matrix = np.array([
        [math.cos(theta), math.sin(theta) * math.sin(phi), math.cos(phi) * math.sin(phi)],
        [0, math.cos(phi), -math.sin(phi)],
        [-math.sin(theta), math.cos(theta) * math.sin(phi), math.cos(theta) * math.cos(phi)]
    ])

    return coordinates, rotation_matrix

def calculate_leg_vectors(base_motors, platform_motors, coordinates, rotation_matrix):
    leg_vectors = np.empty((motors, 3))
    transformed_points = np.empty((motors, 3))
    
    for i in range(motors):
        rotated_point = np.dot(rotation_matrix, platform_motors[i])
        transformed_point = rotated_point + coordinates
        transformed_points[i] = transformed_point
        leg_vectors[i] = transformed_point - base_motors[i]
    
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
    
    for i in range(motors):
        ax.plot([base_motors[i, 0], transformed_points[i, 0]],
                [base_motors[i, 1], transformed_points[i, 1]],
                [base_motors[i, 2], transformed_points[i, 2]], 'g-')
    
    base_circle_x, base_circle_y, base_circle_z = generate_circle(radius, 0)
    ax.plot(base_circle_x, base_circle_y, base_circle_z, 'b--', label='Base Circle')
    
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.legend()
    plt.show()

def calculateVectorMagnitude(arr, power):
    x = arr[0] ** 2
    y = arr[1] ** 2
    z = arr[2] ** 2
    return (math.sqrt(x + y + z)) ** power

def calculate_alpha(e_k, f_k, g_k):
    sqrt_term = math.sqrt(e_k**2 + f_k**2)

    if -1 <= (g_k / sqrt_term) <= 1:
        sin_term = math.asin(g_k / sqrt_term)
        atan2_term = math.atan2(f_k, e_k)
        alpha_k = sin_term - atan2_term
        
        return alpha_k
    else:
        return -1


def calculateServoAngles(servo_vectors, flapVector, beta):
    servoAngleList = []
    flapMagnitude = calculateVectorMagnitude(flapVector, 1)
    for servo in servo_vectors:
        Ek = 2*flapMagnitude*servo[2]
        Fk = 2*flapMagnitude*((math.cos(math.radians(beta)))*servo[0] + math.sin(math.radians(beta))*servo[1])
        Gk = calculateVectorMagnitude(servo, 2) - (leg_length**2 - calculateVectorMagnitude(flapVector, 2))
        servoAngle = calculate_alpha(Ek, Fk, Gk)
        if servoAngle == -1 or -90 > servoAngle > 90: # Angular constraints of servos
            print("Servo Position not Possible")
            return []
        else: 
            servoAngleList.append(servoAngle)
    
    servoAnglesDegrees = [math.degrees(angle) for angle in servoAngleList]
    return servoAnglesDegrees

if __name__ == '__main__':
    coordinates, rotation_matrix = input_parameters()
    leg_vectors, transformed_points = calculate_leg_vectors(base_motors, platform_motors, coordinates, rotation_matrix)
    servoAngles = calculateServoAngles(leg_vectors, flapVector, beta)
    if servoAngles != []:
        plot_stewart_platform(base_motors, transformed_points)
        print(servoAngles)
