import numpy as np
import gymnasium as gym
import rclpy

from prodapt.envs.ur10_ros_interface.joint_states_subscriber import (
    JointStatesSubscriber,
)
from prodapt.envs.ur10_ros_interface.force_subscriber import ForceSubscriber
from prodapt.envs.ur10_ros_interface.movel_publisher import MovelPublisher
from prodapt.envs.ur10_ros_interface.joints_publisher import JointsPublisher


class UR10Env(gym.Env):
    def __init__(self, controller, obs_dict, simulator):
        rclpy.init()
        self.joint_state_subscriber = JointStatesSubscriber(obs_dict)
        self.force_subscriber = ForceSubscriber(obs_dict)

        if controller == "movel":
            self.command_publisher = MovelPublisher()
        elif controller == "joints":
            self.command_publisher = JointsPublisher(simulator=simulator)

        self.last_joint_pos = None
        self.reset_joint_pos = np.array(
            [[0.45045, -1.79302, -2.73423, -0.18513, 1.5708, -1.12034]]
        )
        self.reset_rot_6d = [1, 0, 0, 0, -1, 0]

    def close(self):
        self.joint_state_subscriber.destroy_node()
        self.force_subscriber.destroy_node()
        self.command_publisher.destroy_node()
        rclpy.shutdown()

    def reset(self):
        self.command_publisher.send_action(
            action=[0.4, 0, 0.1, 1, 0, 0, 0, -1, 0],
            duration=10,
            last_joint_pos=self.reset_joint_pos,
        )
        obs = self._get_latest_observation()
        return obs, {}

    def step(self, action):
        action = np.concatenate((action, self.reset_rot_6d))
        if self.last_joint_pos is None:
            self._get_latest_observation()
        self.command_publisher.send_action(
            action=action, duration=0.1, last_joint_pos=self.last_joint_pos
        )
        obs = self._get_latest_observation()

        done = np.linalg.norm(obs[:2] - np.array([-1.2, 0])) < 0.025

        return obs, 0, done, False, {}

    def seed(self, seed=None):
        pass

    def _get_latest_observation(self):
        self.joint_state_subscriber.last_obs = None
        while self.joint_state_subscriber.last_obs is None:
            rclpy.spin_once(self.joint_state_subscriber)

        self.force_subscriber.last_obs = None
        while self.force_subscriber.last_obs is None:
            rclpy.spin_once(self.force_subscriber)

        obs = np.concatenate(
            (self.joint_state_subscriber.last_obs, self.force_subscriber.last_obs)
        )

        self.last_joint_pos = self.joint_state_subscriber.last_joint_pos
        return obs
