import numpy as np
import gymnasium as gym
import rclpy

from prodapt.envs.ur10_ros_interface.joint_states_subscriber import (
    JointStatesSubscriber,
)
from prodapt.envs.ur10_ros_interface.movel_publisher import MovelPublisher
from prodapt.envs.ur10_ros_interface.joints_publisher import JointsPublisher


class UR10Env(gym.Env):
    def __init__(self, controller, obs_dict):
        rclpy.init()
        self.joint_state_subscriber = JointStatesSubscriber(obs_dict)

        if controller == "movel":
            self.command_publisher = MovelPublisher()
        elif controller == "joints":
            self.command_publisher = JointsPublisher()

        self.last_joint_pos = None
        self.reset_joint_pos = np.array(
            [[0.2948, -1.4945, -2.1491, -1.0693, 1.5693, -1.2760]]
        )

    def close(self):
        self.joint_state_subscriber.destroy_node()
        self.command_publisher.destroy_node()
        rclpy.shutdown()

    def reset(self):
        self.command_publisher.send_action(
            action=[0.6, 0, 0.4, 1, 0, 0, 0, -1, 0],
            duration=10,
            last_joint_pos=self.reset_joint_pos,
        )
        obs = self._get_latest_observation()
        return obs, {}

    def step(self, action):
        if self.last_joint_pos is None:
            self._get_latest_observation()
        self.command_publisher.send_action(
            action=action, duration=0.1, last_joint_pos=self.last_joint_pos
        )
        obs = self._get_latest_observation()
        return obs, 0, False, False, {}

    def seed(self, seed=None):
        pass

    def _get_latest_observation(self):
        self.joint_state_subscriber.last_obs = None
        while self.joint_state_subscriber.last_obs is None:
            rclpy.spin_once(self.joint_state_subscriber)
        obs = self.joint_state_subscriber.last_obs
        self.last_joint_pos = self.joint_state_subscriber.last_joint_pos
        return obs
