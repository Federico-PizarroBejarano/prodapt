from cmath import acos, asin
import numpy as np

from prodapt.utils.rotation_utils import bound_angles

# Denavit–Hartenberg parameters
global d1, a2, a3, d4, d5, d6
d1 = 0.1807
a2 = -0.6127
a3 = -0.57155
d4 = 0.17415
d5 = 0.11985
d6 = 0.11655

global d, a, alph

d = np.matrix([d1, 0, 0, d4, d5, d6])
a = np.matrix([0, a2, a3, 0, 0, 0])
alph = np.matrix([np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0])


def inverse_kinematics(desired_pos):
    th = np.matrix(np.zeros((6, 8)))
    P_05 = desired_pos * np.matrix([0, 0, -d6, 1]).T - np.matrix([0, 0, 0, 1]).T

    # **** theta1 ****
    psi = np.arctan2(P_05[2 - 1, 0], P_05[1 - 1, 0])
    phi = acos(
        d4 / (P_05[2 - 1, 0] * P_05[2 - 1, 0] + P_05[1 - 1, 0] * P_05[1 - 1, 0]) ** 0.5
    ).real
    # The two solutions for theta1 correspond to the shoulder
    # being either left or right
    th[0, 0:4] = np.pi / 2 + psi + phi
    th[0, 4:8] = np.pi / 2 + psi - phi
    th = th.real

    # **** theta5 ****
    cl = [0, 4]  # wrist up or down
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(AH(1, th, c))
        T_16 = T_10 * desired_pos
        th[4, c : c + 2] = acos((T_16[2, 3] - d4) / d6).real
        th[4, c + 2 : c + 4] = -acos((T_16[2, 3] - d4) / d6).real

    th = th.real

    # **** theta6 ****
    cl = [0, 2, 4, 6]
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(AH(1, th, c))
        T_16 = np.linalg.inv(T_10 * desired_pos)
        # theta6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.
        if np.sin(th[4, c]) == 0 or T_16[1, 2] == 0 or T_16[0, 2] == 0:
            th[5, c : c + 2] = float("inf")
        else:
            th[5, c : c + 2] = np.arctan2(
                (-T_16[1, 2] / np.sin(th[4, c])), (T_16[0, 2] / np.sin(th[4, c]))
            )

    th = th.real

    # **** theta3 ****
    cl = [0, 2, 4, 6]
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(AH(1, th, c))
        T_65 = AH(6, th, c)
        T_54 = AH(5, th, c)
        T_14 = (T_10 * desired_pos) * np.linalg.inv(T_54 * T_65)
        P_13 = T_14 * np.matrix([0, -d4, 0, 1]).T - np.matrix([0, 0, 0, 1]).T
        t3 = acos((np.linalg.norm(P_13) ** 2 - a2**2 - a3**2) / (2 * a2 * a3))
        th[2, c] = t3.real
        th[2, c + 1] = -t3.real

    # **** theta2 and theta 4 ****
    cl = [0, 1, 2, 3, 4, 5, 6, 7]
    for i in range(0, len(cl)):
        c = cl[i]
        T_10 = np.linalg.inv(AH(1, th, c))
        T_65 = np.linalg.inv(AH(6, th, c))
        T_54 = np.linalg.inv(AH(5, th, c))
        T_14 = (T_10 * desired_pos) * T_65 * T_54
        P_13 = T_14 * np.matrix([0, -d4, 0, 1]).T - np.matrix([0, 0, 0, 1]).T

        # theta 2
        th[1, c] = (
            -np.arctan2(P_13[1], -P_13[0])
            + asin(a3 * np.sin(th[2, c]) / np.linalg.norm(P_13)).real
        )
        # theta 4
        T_32 = np.linalg.inv(AH(3, th, c))
        T_21 = np.linalg.inv(AH(2, th, c))
        T_34 = T_32 * T_21 * T_14
        th[3, c] = np.arctan2(T_34[1, 0], T_34[0, 0])
    th = th.real

    th = bound_angles(th)

    return th


def AH(n, th, c=0):
    T_a = np.matrix(np.identity(4), copy=False)
    T_a[0, 3] = a[0, n - 1]
    T_d = np.matrix(np.identity(4), copy=False)
    T_d[2, 3] = d[0, n - 1]

    Rzt = np.matrix(
        [
            [np.cos(th[n - 1, c]), -np.sin(th[n - 1, c]), 0, 0],
            [np.sin(th[n - 1, c]), np.cos(th[n - 1, c]), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ],
        copy=False,
    )

    Rxa = np.matrix(
        [
            [1, 0, 0, 0],
            [0, np.cos(alph[0, n - 1]), -np.sin(alph[0, n - 1]), 0],
            [0, np.sin(alph[0, n - 1]), np.cos(alph[0, n - 1]), 0],
            [0, 0, 0, 1],
        ],
        copy=False,
    )

    A_i = T_d * Rzt * T_a * Rxa

    return A_i


def forward_kinematics(joint_pos):
    A_1 = AH(1, joint_pos)
    A_2 = AH(2, joint_pos)
    A_3 = AH(3, joint_pos)
    A_4 = AH(4, joint_pos)
    A_5 = AH(5, joint_pos)
    A_6 = AH(6, joint_pos)

    T_06 = A_1 * A_2 * A_3 * A_4 * A_5 * A_6

    return np.array(T_06)


def choose_best_ik(IK, curr_joint_pos):
    bounded_curr = bound_angles(curr_joint_pos)
    diff_between_ik_curr = bound_angles(IK.T - np.squeeze(bounded_curr))
    normed_dist = list(np.linalg.norm(diff_between_ik_curr, axis=1))
    min_idx = normed_dist.index(min(normed_dist))
    best_IK = IK[:, min_idx]
    best_IK = np.array(best_IK).squeeze()
    return best_IK


if __name__ == "__main__":
    joint_pos = np.array([[0.2948, -1.4945, -2.1491, -1.0693, 1.5693, -1.2760]])
    ee_pose = forward_kinematics(joint_pos.T)
    print(np.round(ee_pose, 3))
    IK = inverse_kinematics(ee_pose)
    print(np.round(joint_pos, 3))
    print(np.round(IK, 3))
    best_IK = choose_best_ik(IK, joint_pos)
    print(np.round(best_IK.T, 3))
