import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped


class ForcePublisher(Node):
    def __init__(self):
        rclpy.init()
        super().__init__("force_publisher")
        self.publisher = self.create_publisher(
            WrenchStamped, "/force_torque_sensor_broadcaster/wrench", 10
        )

    def publish_force(self, force_torque):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = ""

        force_torque_msg = WrenchStamped()
        force_torque_msg.header = header
        force_torque_msg.wrench = Wrench()
        force_torque = force_torque.tolist()

        force_torque_msg.wrench.force = Vector3()
        force_torque_msg.wrench.force.x = force_torque[0]
        force_torque_msg.wrench.force.y = force_torque[1]
        force_torque_msg.wrench.force.z = force_torque[2]

        force_torque_msg.wrench.torque = Vector3()
        force_torque_msg.wrench.torque.x = force_torque[3]
        force_torque_msg.wrench.torque.y = force_torque[4]
        force_torque_msg.wrench.torque.z = force_torque[5]

        self.publisher.publish(force_torque_msg)
