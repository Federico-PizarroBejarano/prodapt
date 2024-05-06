import time

from rclpy.node import Node
from rclpy.time import Duration

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from prodapt.utils.kinematics_utils import (
    inverse_kinematics,
    choose_best_ik,
)
from prodapt.utils.rotation_utils import (
    get_T_matrix,
    matrix_to_quaternion,
    rotation_6d_to_matrix,
)


class JointsPublisher(Node):
    def __init__(self):
        super().__init__("joints_publisher")
        self.publisher = self.create_publisher(
            JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
        )

        self.ordered_link_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

    def send_action(self, action, duration, last_joint_pos):
        quat = matrix_to_quaternion(rotation_6d_to_matrix(action[3:]))
        transformation_matrix = get_T_matrix(action[:3], quat)
        IK = inverse_kinematics(transformation_matrix)
        best_IK = choose_best_ik(IK, last_joint_pos)

        joint_command = JointTrajectory()
        joint_command.header = Header()
        joint_command.header.stamp = self.get_clock().now().to_msg()
        joint_command.header.frame_id = ""
        joint_command.joint_names = self.ordered_link_names
        joint_command.points = [JointTrajectoryPoint()]
        joint_command.points[0].positions = [float(elem) for elem in best_IK]
        joint_command.points[0].time_from_start = Duration(seconds=duration).to_msg()

        self.publisher.publish(joint_command)
        time.sleep(duration)
