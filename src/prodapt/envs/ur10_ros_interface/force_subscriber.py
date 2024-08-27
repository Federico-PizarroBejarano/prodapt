import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


class ForceSubscriber(Node):
    def __init__(self, obs_list):
        super().__init__("force_subscriber")
        self.obs_list = obs_list
        self.subscription = self.create_subscription(
            WrenchStamped,
            "/force_torque_sensor_broadcaster/wrench",
            self.listener_callback,
            10,
        )

        self.last_obs = None

    def listener_callback(self, msg):
        self.last_obs = []

        if "force" in self.obs_list:
            self.last_obs.append(
                [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
            )
        if "torque" in self.obs_list:
            self.last_obs.append(
                [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
            )
        if "torque2" in self.obs_list:
            self.last_obs.append([msg.wrench.torque.y, msg.wrench.torque.z])

        self.last_obs = np.concatenate(self.last_obs)
