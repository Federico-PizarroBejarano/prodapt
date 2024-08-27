from rclpy.node import Node
from rclpy.time import Duration

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from prodapt.utils.kinematics_utils import inverse_kinematics, choose_best_ik
from prodapt.utils.rotation_utils import (
    get_T_matrix,
    matrix_to_quaternion,
    rotation_6d_to_matrix,
)


class JointsPublisher(Node):
    def __init__(self, action_list, interface, base_command):
        super().__init__("joints_publisher")

        self.action_list = action_list
        self.interface = interface
        self.base_command = base_command

        if self.interface == "ur-driver":
            self.publisher = self.create_publisher(
                JointTrajectory,
                "/scaled_joint_trajectory_controller/joint_trajectory",
                10,
            )
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

    def send_action(self, action, duration, last_joint_pos):
        applied_action = self.base_command.copy()
        if "commanded_ee_position" in self.action_list:
            applied_action[:3] = action[:3]
        if "commanded_ee_position_xy" in self.action_list:
            applied_action[:2] = action[:2]
        if "commanded_ee_rotation_6d" in self.action_list:
            applied_action[3:] = action[3:]
        quat = matrix_to_quaternion(rotation_6d_to_matrix(applied_action[3:]))
        transformation_matrix = get_T_matrix(applied_action[:3], quat)
        IK = inverse_kinematics(transformation_matrix)
        best_IK = choose_best_ik(IK, last_joint_pos)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = ""

        if self.interface == "ur-driver":
            joint_command = JointTrajectory()
            joint_command.header = header
            joint_command.joint_names = self.ordered_link_names
            joint_command.points = [JointTrajectoryPoint()]
            joint_command.points[0].positions = [float(elem) for elem in best_IK]
            joint_command.points[0].time_from_start = Duration(
                seconds=duration
            ).to_msg()
        elif self.interface == "isaacsim":
            joint_command = JointState()
            joint_command.header = header
            joint_command.name = self.ordered_link_names
            joint_command.position = [float(elem) for elem in best_IK]

        self.publisher.publish(joint_command)
