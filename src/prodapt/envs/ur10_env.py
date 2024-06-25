import numpy as np
import gymnasium as gym
import rclpy

from prodapt.utils.keypoint_manager import KeypointManager
from prodapt.envs.ur10_ros_interface.joint_states_subscriber import (
    JointStatesSubscriber,
)
from prodapt.envs.ur10_ros_interface.force_subscriber import ForceSubscriber
from prodapt.envs.ur10_ros_interface.movel_publisher import MovelPublisher
from prodapt.envs.ur10_ros_interface.joints_publisher import JointsPublisher


class UR10Env(gym.Env):
    def __init__(self, controller, action_list, obs_list, simulator, keypoint_args):
        rclpy.init()
        self.joint_state_subscriber = JointStatesSubscriber(obs_list)
        self.force_subscriber = ForceSubscriber(obs_list)

        if controller == "movel":
            self.base_command = [0.4, 0, 0.1, 3.14, 0, 0]
            self.command_publisher = MovelPublisher(action_list=action_list)
        elif controller == "joints":
            self.base_command = [0.4, 0, 0.1, 1, 0, 0, 0, -1, 0]
            self.command_publisher = JointsPublisher(
                action_list=action_list,
                simulator=simulator,
                base_command=self.base_command,
            )

        self.last_joint_pos = None
        self.reset_joint_pos = np.array(
            [[0.45045, -1.79302, -2.73423, -0.18513, 1.5708, -1.12034]]
        )

        self.use_force_obs = False
        self.use_keypoint_obs = False
        if "force" in obs_list or "torque" in obs_list or "torque2" in obs_list:
            self.use_force_obs = True
        if np.any(["keypoint" in key for key in obs_list]):
            self.use_keypoint_obs = True

        if self.use_keypoint_obs:
            self.keypoint_manager = KeypointManager(**keypoint_args)

    def close(self):
        self.joint_state_subscriber.destroy_node()
        self.force_subscriber.destroy_node()
        self.command_publisher.destroy_node()
        rclpy.shutdown()

    def reset(self):
        if self.use_keypoint_obs:
            self.keypoint_manager.reset()

        self.command_publisher.send_action(
            action=self.base_command,
            duration=3,
            last_joint_pos=self.reset_joint_pos,
        )

        obs, info = self._get_latest_observation()
        assert info["kp_added"] is False
        return obs, info

    def step(self, action):
        if self.last_joint_pos is None:
            self._get_latest_observation()
        self.command_publisher.send_action(
            action=action, duration=0.1, last_joint_pos=self.last_joint_pos
        )
        obs, info = self._get_latest_observation()

        done = self._get_done(obs)
        reward = -1 if not done else 1000

        return obs, reward, done, False, info

    def _get_done(self, obs):
        done = np.linalg.norm(obs[:2] - np.array([1.2, 0])) < 0.025
        return done

    def seed(self, seed=None):
        pass

    def _get_latest_observation(self):
        info = {}
        kp_added = False

        self.joint_state_subscriber.last_obs = None
        while self.joint_state_subscriber.last_obs is None:
            rclpy.spin_once(self.joint_state_subscriber)
        obs = self.joint_state_subscriber.last_obs

        if self.use_force_obs:
            self.force_subscriber.last_obs = None
            while self.force_subscriber.last_obs is None:
                rclpy.spin_once(self.force_subscriber)
            obs = np.concatenate((obs, self.force_subscriber.last_obs))

        if self.use_keypoint_obs:
            kp_added = self.keypoint_manager.add_keypoint(
                self.joint_state_subscriber.last_obs,
                self.force_subscriber.last_full_msg,
            )
            for kp in range(self.keypoint_manager.num_keypoints):
                obs = np.concatenate(
                    (
                        obs,
                        self.keypoint_manager.all_keypoints[kp][0],
                        self.keypoint_manager.all_keypoints[kp][1],
                    )
                )

        info = {"kp_added": kp_added}

        self.last_joint_pos = self.joint_state_subscriber.last_joint_pos
        return obs, info
