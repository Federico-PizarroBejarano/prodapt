import numpy as np

from rclpy.node import Node

from std_msgs.msg import Header, Float64MultiArray
from sensor_msgs.msg import JointState

from prodapt.utils.kinematics_utils import inverse_kinematics, choose_best_ik
from prodapt.utils.rotation_utils import (
    get_T_matrix,
    matrix_to_quaternion,
    rotation_6d_to_matrix,
    real_exp_transform,
    bound_angles,
)


class JointsPublisher(Node):
    def __init__(self, action_list, interface, base_command):
        super().__init__("joints_publisher")

        self.action_list = action_list
        self.interface = interface
        self.base_command = base_command

        if self.interface == "ur-driver":
            self.publisher = self.create_publisher(
                Float64MultiArray, "/forward_velocity_controller/commands", 10
            )
            self.prev_velocity = [0.0] * 6
        if self.interface == "isaacsim":
            self.publisher = self.create_publisher(JointState, "/joint_command", 10)

        self.ordered_link_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.snap_speedup = 2.0

    def send_action(self, action, last_joint_pos, z_offset=0.0, bypass_acc_limit=False):
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

        if self.interface == "ur-driver":
            joint_command = Float64MultiArray()
            joint_vels = bound_angles(best_IK - last_joint_pos)
            joint_vels = np.clip(joint_vels * self.snap_speedup, -0.05, 0.05)
            joint_command.data = list(joint_vels)

            # Limit acceleration
            if not bypass_acc_limit:
                vel_diff = np.clip(joint_vels - self.prev_velocity, -0.025, 0.025)
                joint_command.data = list(self.prev_velocity + vel_diff)
                self.prev_velocity += vel_diff
        elif self.interface == "isaacsim":
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = ""
            joint_command = JointState()
            joint_command.header = header
            joint_command.name = self.ordered_link_names
            joint_command.position = [float(elem) for elem in best_IK]

        self.publisher.publish(joint_command)
        return best_IK

    def send_zeros(self):
        if self.interface == "ur-driver":
            joint_command = Float64MultiArray()
            joint_command.data = [0.0] * 6

            self.publisher.publish(joint_command)
