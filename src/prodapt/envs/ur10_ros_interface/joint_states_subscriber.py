import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState

from prodapt.utils.kinematics_utils import forward_kinematics
from prodapt.utils.rotation_utils import matrix_to_rotation_6d


class JointStatesSubscriber(Node):
    def __init__(self, obs_list):
        super().__init__("joint_states_subscriber")
        self.obs_list = obs_list
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.listener_callback, 10
        )

        self.last_obs = None
        self.last_joint_pos = None
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
        self.last_obs = []

        joint_pos = np.array(msg.position)[reorder]

        if "joint_pos" in self.obs_list:
            self.last_obs.append(joint_pos)
        if "joint_vel" in self.obs_list:
            self.last_obs.append(np.array(msg.velocity)[reorder])
        if "joint_eff" in self.obs_list:
            self.last_obs.append(np.array(msg.effort)[reorder])
        if "ee_position" in self.obs_list:
            T_matrix = forward_kinematics(joint_pos.reshape(6, 1))
            translation = T_matrix[:3, 3].squeeze()
            self.last_obs.append(translation)
        if "ee_position_xy" in self.obs_list:
            T_matrix = forward_kinematics(joint_pos.reshape(6, 1))
            translation = T_matrix[:3, 3].squeeze()
            self.last_obs.append(translation[:2])
        if "ee_rotation_6d" in self.obs_list:
            T_matrix = forward_kinematics(joint_pos.reshape(6, 1))
            rotation_6d = matrix_to_rotation_6d(T_matrix[:3, :3]).squeeze()
            self.last_obs.append(rotation_6d)

        self.last_obs = np.concatenate(self.last_obs)
        self.last_joint_pos = joint_pos
