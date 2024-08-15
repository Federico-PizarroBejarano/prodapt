import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np

from spacenav_converter.kinematics_utils import (
    inverse_kinematics,
    choose_best_ik,
)
from spacenav_converter.rotation_utils import get_T_matrix, bound_angles


class ConverterToJoints(Node):
    def __init__(self):
        super().__init__("converter_to_joints")

        self.declare_parameter(name="simulator", value="ursim")
        self.simulator = (
            self.get_parameter("simulator").get_parameter_value().string_value
        )

        if self.simulator == "ursim":
            self.publisher = self.create_publisher(
                Float64MultiArray,
                "/forward_velocity_controller/commands",
                10,
            )
        if self.simulator == "isaacsim":
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
        self.spacenav_to_delta_const = 0.4

        self.last_command = None
        self.last_joint_pos = None
        self.dt = 0.1
        self.timer_period = self.dt
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
            linear, _ = self.last_command

            t = self.tf_buffer.lookup_transform(
                self.from_frame_rel, self.to_frame_rel, rclpy.time.Time()
            )

            quat_curr = [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ]

            translation = [
                t.transform.translation.x
                + self.timer_period * self.spacenav_to_delta_const * linear.x,
                t.transform.translation.y
                + self.timer_period * self.spacenav_to_delta_const * linear.y,
                t.transform.translation.z
                + self.timer_period * self.spacenav_to_delta_const * linear.z,
            ]
            transformation_matrix = get_T_matrix(translation, quat_curr)
            IK = inverse_kinematics(transformation_matrix)
            best_IK = choose_best_ik(IK, self.last_joint_pos)

            command = Float64MultiArray()
            command.data = [(float(best_IK[i]) - self.last_joint_pos[i]) for i in range(6)]

            self.publisher.publish(command)

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
