import numpy as np

def normalize_vector(v):
    return v / np.linalg.norm(v)

def cross_product(a, b):
    return np.cross(a, b)

def dot_product(a, b):
    return np.dot(a, b)

def rodrigues_rotation(axis, angle):
    axis = normalize_vector(axis)
    x, y, z = axis
    c = np.cos(angle)
    s = np.sin(angle)
    C = 1 - c

    return np.array([[x*x*C + c, x*y*C - z*s, x*z*C + y*s],
                     [x*y*C + z*s, y*y*C + c, y*z*C - x*s],
                     [x*z*C - y*s, y*z*C + x*s, z*z*C + c]])

def align_gravity_with_z_axis(accelerometer_vector):
    a_normalized = normalize_vector(accelerometer_vector)
    target_gravity = np.array([0, 0, 1])

    rotation_axis = cross_product(a_normalized, target_gravity)
    theta = np.arccos(dot_product(a_normalized, target_gravity))

    R = rodrigues_rotation(rotation_axis, theta)

    # Adjust the rotation matrix to align the z-axis with gravity
    R[[0, 1, 2], :] = R[[2, 0, 1], :]

    return R

# Example usage
accelerometer_vector = np.array([ax, ay, az])  # Replace with actual accelerometer values
normalized_vector = normalize_vector(accelerometer_vector)
rotation_matrix = align_gravity_with_z_axis(accelerometer_vector)

print("Normalized accelerometer vector:")
print(normalized_vector)

print("\nRotation matrix:")
print(rotation_matrix)
