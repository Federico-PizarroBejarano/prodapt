import numpy as np
from tf_transformations import quaternion_matrix, quaternion_from_matrix


def get_T_matrix(translation, quaternion):
    translation = np.squeeze(translation)
    T_mat = quaternion_matrix(quaternion)
    T_mat[0:3, 3] = translation
    return T_mat


def bound_angles(list_of_angles):
    list_of_angles = np.array(list_of_angles)
    return (list_of_angles + np.pi) % (2 * np.pi) - np.pi


def axis_angle_to_quaternion(axis_angle):
    axis_angle = np.array(axis_angle)
    angle = np.linalg.norm(axis_angle)
    w = np.cos(angle * 0.5)
    s = (1 - w**2) ** 0.5
    if s < 0.001:
        quat = np.concatenate([axis_angle, np.array([w])])
        return quat / np.linalg.norm(quat)
    else:
        quat = np.concatenate([axis_angle * s / angle, np.array([w])])
        return quat / np.linalg.norm(quat)


def quaternion_to_axis_angle(quaternion):
    quaternion = np.array(quaternion)
    x, y, z, w = quaternion
    angle = np.arccos(w) * 2
    s = (1 - w**2) ** 0.5
    if s < 0.001:
        return np.array([x, y, z])
    else:
        return np.array([x * angle / s, y * angle / s, z * angle / s])


def rotation_6d_to_matrix(rotation_6d):
    rotation_6d = np.array(rotation_6d)
    a1, a2 = rotation_6d[:3], rotation_6d[3:]
    b1 = normalize(a1)
    b2 = a2 - np.dot(b1, a2) * b1
    b2 = normalize(b2)
    b3 = np.cross(b1, b2)
    return np.stack((b1, b2, b3), axis=0)


def matrix_to_rotation_6d(matrix):
    return matrix[:2, :].copy().reshape((6,))


def matrix_to_quaternion(matrix):
    T = np.eye(4)
    T[:3, :3] = matrix
    return np.array(quaternion_from_matrix(T))


def quaternion_to_matrix(quaternion):
    return quaternion_matrix(quaternion)[:3, :3]


def matrix_to_axis_angle(matrix):
    return quaternion_to_axis_angle(matrix_to_quaternion(matrix))


def axis_angle_to_matrix(axis_angle):
    return quaternion_to_matrix(axis_angle_to_quaternion(axis_angle))[:3, :3]


def rotation_6d_to_axis_angle(rotation_6d):
    return matrix_to_axis_angle(rotation_6d_to_matrix(rotation_6d))


def axis_angle_to_rotation_6d(axis_angle):
    return matrix_to_rotation_6d(axis_angle_to_matrix(axis_angle))


def normalize(vector):
    vector = np.array(vector)
    magnitude = np.linalg.norm(vector)
    return vector / magnitude


def normalize_axis_angle(axis_angle):
    axis_angle = np.array(axis_angle)
    angle = np.linalg.norm(axis_angle)
    if angle == 0:
        return axis_angle
    else:
        new_angle = bound_angles(angle)
        axis_angle = axis_angle / angle * new_angle
        return axis_angle


if __name__ == "__main__":
    unnormalized_axis_angle = np.array(
        [
            np.random.rand() * 2 * np.pi - np.pi,
            np.random.rand() * 2 * np.pi - np.pi,
            np.random.rand() * 2 * np.pi - np.pi,
        ]
    )
    print("unnormalized axis_angle", unnormalized_axis_angle)
    axis_angle = normalize_axis_angle(unnormalized_axis_angle)
    print("axis_angle 1", axis_angle)
    quat = axis_angle_to_quaternion(axis_angle)
    print("quat 1", quat)
    axis_angle2 = quaternion_to_axis_angle(quat)
    print("axis_angle 2", axis_angle2)
    matrix = axis_angle_to_matrix(axis_angle)
    print("matrix 1", matrix)
    sixd = matrix_to_rotation_6d(matrix)
    print("sixd 1", sixd)
    matrix2 = rotation_6d_to_matrix(sixd)
    print("matrix 2", matrix2)
    quat2 = matrix_to_quaternion(matrix2)
    print("quat 2", quat2)
    axis_angle3 = quaternion_to_axis_angle(quat2)
    print("axis_angle 3", axis_angle3)

    assert np.linalg.norm(axis_angle - axis_angle2) < 1e-6
    assert np.linalg.norm(matrix - matrix2) < 1e-6
    assert np.linalg.norm(quat - quat2) < 1e-6 or np.linalg.norm(quat + quat2) < 1e-6
    assert np.linalg.norm(axis_angle - axis_angle3) < 1e-6
