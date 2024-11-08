from inverseKinematics import input_parameters, calculate_leg_vectors, calculateStepperAngles
from RPI_interface import writeInverseKinematics
import config

if __name__ == '__main__':
    while True: 
        pitch, roll = map(float, input("Enter desired pitch and roll ").split())
        coordinates, rotation_matrix = input_parameters(pitch, roll)
        leg_vectors, transformed_points = calculate_leg_vectors(config.base_motors, config.platform_motors, coordinates, rotation_matrix)
        stepperAngles = calculateStepperAngles(leg_vectors)
        writeInverseKinematics(stepperAngles) # TODO: Add acknowledgement from Arduino