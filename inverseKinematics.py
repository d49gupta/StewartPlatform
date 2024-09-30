import numpy as np
import math
import matplotlib.pyplot as plt

motors = 6
radius = 5
leg_height = 5

section_angle = 360 / motors
base_motors = np.empty((motors, 3))
platform_motors = np.empty((motors, 3))

for i in range(motors):
    x = radius * math.cos(math.radians(i * section_angle))
    y = radius * math.sin(math.radians(i * section_angle))
    base_motors[i] = [x, y, 0]  # Base points at z = 0
    platform_motors[i] = [x, y, leg_height]  # Platform points at height leg_height

def input_parameters():
    Tx, Ty, Tz = map(float, input("Enter desired coordinates (Tx Ty Tz): ").split())
    coordinates = np.array([Tx, Ty, Tz])

    target_magnitude = np.linalg.norm(coordinates)
    n_x, n_y, n_z = coordinates / target_magnitude    
    pitch = np.arctan2(n_y, n_z)  # Angle in radians
    roll = np.arctan2(n_x, n_z)   # Angle in radians    
    theta = np.degrees(pitch)
    phi = np.degrees(roll)
    
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
        transformed_points[i] = transformed_point  # Store the transformed point
    
        leg_vectors[i] = transformed_point - base_motors[i]
    
    return leg_vectors, transformed_points

def generate_circle(radius, z, num_points=100):
    angles = np.linspace(0, 2 * np.pi, num_points)
    x = radius * np.cos(angles)
    y = radius * np.sin(angles)
    z = np.full_like(x, z)
    return x, y, z

def plot_stewart_platform(base_motors, platform_motors, transformed_points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(base_motors[:, 0], base_motors[:, 1], base_motors[:, 2], c='b', label='Base Points')
    ax.scatter(transformed_points[:, 0], transformed_points[:, 1], transformed_points[:, 2], c='r', label='Platform Points')
    
    # Draw legs
    for i in range(motors):
        ax.plot([base_motors[i, 0], transformed_points[i, 0]],
                [base_motors[i, 1], transformed_points[i, 1]],
                [base_motors[i, 2], transformed_points[i, 2]], 'g-')
    
    base_circle_x, base_circle_y, base_circle_z = generate_circle(radius, 0)
    ax.plot(base_circle_x, base_circle_y, base_circle_z, 'b--', label='Base Circle')
    platform_circle_x, platform_circle_y, platform_circle_z = generate_circle(radius, transformed_points[0, 2])
    ax.plot(platform_circle_x, platform_circle_y, platform_circle_z, 'r--', label='Platform Circle')
    
    # Set labels and show plot
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.legend()
    plt.show()

if __name__ == '__main__':
    coordinates, rotation_matrix = input_parameters()
    leg_vectors, transformed_points = calculate_leg_vectors(base_motors, platform_motors, coordinates, rotation_matrix)
    plot_stewart_platform(base_motors, platform_motors, transformed_points)
