import numpy as np
import math

# Function to convert quaternion pose to flattened 4x4 matrix
def quaternionToFlattenedMatrix(qw, qx, qy, qz):
    
    matrix = np.zeros((4, 4))
    matrix[0, 0] = 1.0 - 2.0 * (qy * qy + qz * qz)
    matrix[0, 1] = 2.0 * (qx * qy - qz * qw)
    matrix[0, 2] = 2.0 * (qx * qz + qy * qw)
    matrix[0, 3] = 0.0
    matrix[1, 0] = 2.0 * (qx * qy + qz * qw)
    matrix[1, 1] = 1.0 - 2.0 * (qx * qx + qz * qz)
    matrix[1, 2] = 2.0 * (qy * qz - qx * qw)
    matrix[1, 3] = 0.0
    matrix[2, 0] = 2.0 * (qx * qz - qy * qw)
    matrix[2, 1] = 2.0 * (qy * qz + qx * qw)
    matrix[2, 2] = 1.0 - 2.0 * (qx * qx + qy * qy)
    matrix[2, 3] = 0.0
    matrix[3, 0] = 0.0
    matrix[3, 1] = 0.0
    matrix[3, 2] = 0.0
    matrix[3, 3] = 1.0
    return matrix


# Helper function to convert a flattened 4x4 matrix pose to Euler angles
def flattenedMatrixToEulerAngles(matrix):
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    pitch = math.atan2(-matrix[2, 0], math.sqrt(matrix[2, 1]**2 + matrix[2, 2]**2))
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return roll, pitch, yaw




# Function to convert Euler angles to flattened 4x4 matrix and quaternion pose
def eulerAnglesToMatrixAndQuaternion(roll, pitch, yaw):
    # Calculate the trigonometric values
    cos_r = math.cos(roll)
    sin_r = math.sin(roll)
    cos_p = math.cos(pitch)
    sin_p = math.sin(pitch)
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)

    # Create the flattened 4x4 matrix
    matrix = np.zeros((4, 4))
    matrix[0, 0] = cos_y * cos_p
    matrix[0, 1] = -sin_y * cos_r + cos_y * sin_p * sin_r
    matrix[0, 2] = sin_y * sin_r + cos_y * sin_p * cos_r
    matrix[0, 3] = 0.0
    matrix[1, 0] = sin_y * cos_p
    matrix[1, 1] = cos_y * cos_r + sin_y * sin_p * sin_r
    matrix[1, 2] = -cos_y * sin_r + sin_y * sin_p * cos_r
    matrix[1, 3] = 0.0
    matrix[2, 0] = -sin_p
    matrix[2, 1] = cos_p * sin_r
    matrix[2, 2] = cos_p * cos_r
    matrix[2, 3] = 0.0
    matrix[3, 0] = 0.0
    matrix[3, 1] = 0.0
    matrix[3, 2] = 0.0
    matrix[3, 3] = 1.0

    # Calculate the quaternion pose
    qw = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y
    qx = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y
    qy = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y
    qz = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y

    return matrix, [qw, qx, qy, qz]
def eulerToQuaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
     
    return [qw, qx, qy, qz]

# Testing the functions
def test_conversion():
    # Define quaternion pose
    orignal_roll = -3.10604
    orignal_pitch = 0.0978687
    orignal_yaw = -0.0294124
    qw, qx, qy, qz = eulerToQuaternion(orignal_roll,orignal_pitch,orignal_yaw)  # Example quaternion pose

    # Convert quaternion pose to flattened 4x4 matrix
    matrix = quaternionToFlattenedMatrix(qw, qx, qy, qz)

    # Convert flattened 4x4 matrix to Euler angles
    roll, pitch, yaw = flattenedMatrixToEulerAngles(matrix)

    # Convert Euler angles back to flattened 4x4 matrix and quaternion pose
    converted_matrix, converted_quaternion = eulerAnglesToMatrixAndQuaternion(roll, pitch, yaw)

    converted_roll, converted_pitch, converted_yaw = flattenedMatrixToEulerAngles(converted_matrix)
    # Print the results
    print("Original Matrix:")
    print(matrix)
    print("Converted Matrix:")
    print(converted_matrix)
    print("Original Quaternion:")
    print(qw, qx, qy, qz)
    print("Converted Quaternion:")
    print(converted_quaternion)

    # Compare the results
    matrix_diff = np.abs(np.array(matrix) - np.array(converted_matrix))
    quaternion_diff = np.abs(np.array([qw, qx, qy, qz]) - np.array(converted_quaternion))
    print("Matrix Difference:")
    print(matrix_diff)
    print("Quaternion Difference:")
    print(quaternion_diff)

    print("Original Euler Angles (degrees):")
    print(np.degrees([orignal_roll, orignal_pitch, orignal_yaw]))
    print("Converted Euler Angles (degrees):")
    print(np.degrees([converted_roll, converted_pitch, converted_yaw]))

    euler_diff = np.abs(np.degrees([roll, pitch, yaw]) - np.degrees([converted_roll, converted_pitch, converted_yaw]))
    print("Euler Angle Difference (degrees):")
    print(euler_diff)
# Run the testing function
# test_conversion()
radius=0.3
r_step=5

n_step = 360/r_step
c=(((2*math.pi*radius))*r_step/360)
print("circumference = ",c*n_step)
circle_step=0
for i in range(0,int(n_step),1):
  circle_step += c
  print ("iteratio  " , i ,"=",circle_step)
# c_p=5
# t_p=10
# filter=0.005
# for i in range(1000):
#     c_p=filter*t_p+((1-filter)*c_p)
#     print("current position = ",c_p)