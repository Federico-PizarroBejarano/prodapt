import time
import numpy as np

from rclpy.node import Node

from std_msgs.msg import Header, Float64MultiArray
from sensor_msgs.msg import JointState

from prodapt.utils.kinematics_utils import (
    inverse_kinematics,
    choose_best_ik,
)
from prodapt.utils.rotation_utils import (
    get_T_matrix,
    matrix_to_quaternion,
    rotation_6d_to_matrix,
    real_exp_transform
)


class JointsPublisher(Node):
    def __init__(self, action_list, simulator, base_command):
        super().__init__("joints_publisher")

        self.action_list = action_list
        self.simulator = simulator
        self.base_command = base_command

        if self.simulator == "ursim":
            self.publisher = self.create_publisher(
                Float64MultiArray,
                "/forward_velocity_controller/commands",
                10,
            )
            self.prev_velocity = [0.0]*6
        if self.simulator == "isaacsim":
            self.publisher = self.create_publisher(JointState, "/joint_command", 10)

        self.ordered_link_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

    def send_action(self, action, last_joint_pos, duration=0, z_offset=0.0):
        applied_action = self.base_command.copy()
        if "commanded_ee_position" in self.action_list:
            applied_action[:3] = action[:3]
        if "commanded_ee_position_xy" in self.action_list:
            applied_action[:2] = real_exp_transform(action)[:2]
            applied_action[2] += z_offset
        if "commanded_ee_rotation_6d" in self.action_list:
            applied_action[3:] = action[3:]
        quat = matrix_to_quaternion(rotation_6d_to_matrix(applied_action[3:]))
        transformation_matrix = get_T_matrix(applied_action[:3], quat)
        IK = inverse_kinematics(transformation_matrix)
        best_IK = choose_best_ik(IK, last_joint_pos)

        if self.simulator == "ursim":
            joint_command = Float64MultiArray()
            joint_command.data = [(float(best_IK[i]) - np.squeeze(last_joint_pos)[i]) for i in range(6)]
            joint_command.data[-1] = 0.0
            snap_speedup = 2.0
            joint_command.data = [min(max(joint_command.data[i]*snap_speedup, -0.05), 0.05) for i in range(6)]

            # Limit acceleration
            vel_diff = [joint_command.data[i] - self.prev_velocity[i] for i in range(6)]
            clipped_vel_diff = [min(max(vel_diff[i], -0.001), 0.001) for i in range(6)]
            joint_command.data = [self.prev_velocity[i] + clipped_vel_diff[i] for i in range(6)]
            self.prev_velocity = joint_command.data
        elif self.simulator == "isaacsim":
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = ""
            joint_command = JointState()
            joint_command.header = header
            joint_command.name = self.ordered_link_names
            joint_command.position = [float(elem) for elem in best_IK]

        self.publisher.publish(joint_command)
        time.sleep(duration)

    def send_zeros(self):
        if self.simulator == "ursim":
            joint_command = Float64MultiArray()
            joint_command.data = [0.0]*6

            self.publisher.publish(joint_command)
