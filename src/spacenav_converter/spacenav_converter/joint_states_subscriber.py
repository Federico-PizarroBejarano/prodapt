import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatesSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.listener_callback, 10
        )

        self.last_seen_state = None
        self.ordered_link_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

    def listener_callback(self, msg):
        link_names = msg.name
        reorder = [link_names.index(name) for name in self.ordered_link_names]
        pos = np.array(msg.position)[reorder]
        vel = np.array(msg.velocity)[reorder]
        eff = np.array(msg.effort)[reorder]
        self.last_seen_state = [pos, vel, eff]
