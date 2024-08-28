import rclpy
from rclpy.time import Duration
from rclpy.node import Node

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler, quaternion_multiply

import numpy as np

from prodapt.utils.kinematics_utils import inverse_kinematics, choose_best_ik
from prodapt.utils.rotation_utils import get_T_matrix, bound_angles


class ConverterToJoints(Node):
    def __init__(self):
        super().__init__("converter_to_joints")

        self.declare_parameter(name="interface", value="ur-driver")
        self.interface = (
            self.get_parameter("interface").get_parameter_value().string_value
        )

        if self.interface == "ur-driver":
            self.publisher = self.create_publisher(
                JointTrajectory,
                "/scaled_joint_trajectory_controller/joint_trajectory",
                10,
            )
        if self.interface == "isaacsim":
            self.publisher = self.create_publisher(JointState, "/joint_command", 10)

        self.spacenav_subscription = self.create_subscription(
            Twist, "/spacenav/twist", self.spacenav_listener_callback, 10
        )
        self.joint_state_subscription = self.create_subscription(
            JointState, "/joint_states", self.joint_state_listener_callback, 10
        )

        self.to_frame_rel = "tool0"
        self.from_frame_rel = "base"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.spacenav_to_delta_const = 0.2

        self.last_command = None
        self.last_joint_pos = None
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.ordered_link_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

    def timer_callback(self):
        if self.last_command is not None and self.last_joint_pos is not None:
            linear, angular = self.last_command

            t = self.tf_buffer.lookup_transform(
                self.from_frame_rel, self.to_frame_rel, rclpy.time.Time()
            )

            quat_curr = [
                # t.transform.rotation.x,
                # t.transform.rotation.y,
                # t.transform.rotation.z,
                # t.transform.rotation.w,
                1,
                0,
                0,
                0,
            ]

            # # Integrating angular velocities according to quaternion equation: q_new = q_curr + dt/2*w*q_curr
            # quat_angular_vel = quaternion_from_euler(angular.x, angular.y, angular.z)
            # angular_vel_delta = [
            #     self.timer_period * 0.25 * rot / 2
            #     for rot in quat_angular_vel  # 0.25 multiplier added to act more like moveL
            # ]
            # quat_delta = quaternion_multiply(angular_vel_delta, quat_curr)
            # quat_new = [sum(i) for i in zip(quat_curr, quat_delta)]
            # magnitude = sum([i**2 for i in quat_new]) ** 0.5
            # quat_new_normalized = [rot / magnitude for rot in quat_new]

            translation = [
                t.transform.translation.x
                - self.timer_period * self.spacenav_to_delta_const * linear.x,
                t.transform.translation.y
                - self.timer_period * self.spacenav_to_delta_const * linear.y,
                0.1,
                # t.transform.translation.z
                # + self.timer_period * self.spacenav_to_delta_const * linear.z,
            ]
            transformation_matrix = get_T_matrix(translation, quat_curr)
            IK = inverse_kinematics(transformation_matrix)
            best_IK = choose_best_ik(IK, self.last_joint_pos)

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = ""

            if self.interface == "ur-driver":
                joint_command = JointTrajectory()
                joint_command.header = header
                joint_command.joint_names = self.ordered_link_names
                joint_command.points = [JointTrajectoryPoint()]
                joint_command.points[0].positions = [float(elem) for elem in best_IK]
                joint_command.points[0].time_from_start = Duration(seconds=0.1).to_msg()
            elif self.interface == "isaacsim":
                joint_command = JointState()
                joint_command.header = header
                joint_command.name = self.ordered_link_names
                joint_command.position = [float(elem) for elem in best_IK]

            self.publisher.publish(joint_command)

    def spacenav_listener_callback(self, twist_msg):
        linear = twist_msg.linear
        angular = twist_msg.angular

        self.last_command = [linear, angular]

    def joint_state_listener_callback(self, joint_state_msg):
        link_names = joint_state_msg.name
        reorder = [link_names.index(name) for name in self.ordered_link_names]
        pos = np.array(joint_state_msg.position)[reorder]
        self.last_joint_pos = bound_angles(pos)


def main(args=None):
    rclpy.init(args=args)
    converter = ConverterToJoints()
    rclpy.spin(converter)

    converter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
