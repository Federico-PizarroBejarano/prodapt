import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler, quaternion_multiply

from prodapt.utils.rotation_utils import quaternion_to_axis_angle


class ConverterToMovel(Node):
    def __init__(self):
        super().__init__("converter_to_movel")
        self.publisher = self.create_publisher(
            String, "/urscript_interface/script_command", 10
        )
        self.subscription = self.create_subscription(
            Twist, "/spacenav/twist", self.listener_callback, 10
        )

        self.to_frame_rel = "tool0"
        self.from_frame_rel = "base"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_command = None
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        if self.last_command is not None:
            linear, angular = self.last_command

            t = self.tf_buffer.lookup_transform(
                self.from_frame_rel, self.to_frame_rel, rclpy.time.Time()
            )

            quat_curr = [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ]

            # Integrating angular velocities according to quaternion equation: q_new = q_curr + dt/2*w*q_curr
            quat_angular_vel = quaternion_from_euler(angular.x, angular.y, angular.z)
            angular_vel_delta = [
                self.timer_period * rot / 2 for rot in quat_angular_vel
            ]
            quat_delta = quaternion_multiply(angular_vel_delta, quat_curr)
            quat_new = [sum(i) for i in zip(quat_curr, quat_delta)]
            magnitude = sum([i**2 for i in quat_new]) ** 0.5
            quat_new_normalized = [rot / magnitude for rot in quat_new]

            # URScript only accepts rotation commands in axis-angle format
            (ax, ay, az) = quaternion_to_axis_angle(quat_new_normalized)

            urscript_msg = String()
            urscript_msg.data = """
def my_prog():

    movel(p[{0}, {1}, {2}, {3}, {4}, {5}], a=1.2, v=0.25, r=0)

end""".format(
                # Inverting x & y directions to be more intuitive
                t.transform.translation.x - self.timer_period * linear.x,
                t.transform.translation.y - self.timer_period * linear.y,
                t.transform.translation.z + self.timer_period * linear.z,
                ax,
                ay,
                az,
            )

            self.publisher.publish(urscript_msg)

    def listener_callback(self, twist_msg):
        linear = twist_msg.linear
        angular = twist_msg.angular

        self.last_command = [linear, angular]


def main(args=None):
    rclpy.init(args=args)
    converter = ConverterToMovel()
    rclpy.spin(converter)

    converter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
