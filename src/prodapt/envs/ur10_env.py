import numpy as np
import gymnasium as gym
import rclpy
import time

from spacenav_to_movel.spacenav_to_movel.joint_states_subscriber import (
    JointStatesSubscriber,
)
from spacenav_to_movel.spacenav_to_movel.movel_publisher import MovelPublisher


class UR10Env(gym.Env):
    def __init__(self):
        rclpy.init()
        self.joint_state_subscriber = JointStatesSubscriber()
        self.movel_publisher = MovelPublisher()

    def close(self):
        self.joint_state_subscriber.destroy_node()
        self.movel_publisher.destroy_node()
        rclpy.shutdown()

    def reset(self):
        self.movel_publisher.send_action([0.6, 0, 0.4, 3.14, 0, 0])
        time.sleep(10)
        obs = self._get_latest_observation()
        return obs, {}

    def step(self, action):
        self.movel_publisher.send_action(action)
        time.sleep(0.1)
        obs = self._get_latest_observation()
        return obs, 0, False, False, {}

    def seed(self, seed=None):
        pass

    def _get_latest_observation(self):
        self.joint_state_subscriber.last_seen_state = None
        while self.joint_state_subscriber.last_seen_state is None:
            rclpy.spin_once(self.joint_state_subscriber)
        [pos, vel, eff] = self.joint_state_subscriber.last_seen_state

        obs = np.concatenate((pos, vel, eff))
        return obs
