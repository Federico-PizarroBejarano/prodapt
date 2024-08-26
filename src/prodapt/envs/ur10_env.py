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
    def __init__(self, controller, action_list, obs_list, interface, keypoint_args):
        rclpy.init()
        self.name = "ur10"
        self.interface = interface
        self.joint_state_subscriber = JointStatesSubscriber(obs_list)
        self.force_subscriber = ForceSubscriber(obs_list)

        x, y, z = 0.7, 0.0, 0.3

        if controller == "movel":
            self.base_command = [x, y, z, 3.14, 0, 0]
            self.command_publisher = MovelPublisher(action_list=action_list)
        elif controller == "joints":
            self.base_command = [x, y, z, 0, 1, 0, 1, 0, 0]
            self.command_publisher = JointsPublisher(
                action_list=action_list,
                interface=interface,
                base_command=self.base_command,
            )

        self.last_joint_pos = None
        self.last_obs = None
        self.reset_joint_pos = np.array(
            [[0.45045, -1.79302, -2.73423, -0.18513, 1.5708, -1.12034]]
        )

        self.use_force_obs = False
        self.keypoints_in_obs = False
        if "force" in obs_list or "torque" in obs_list or "torque2" in obs_list:
            self.use_force_obs = True
        if np.any(["keypoint" in key for key in obs_list]):
            self.keypoints_in_obs = True

        self.keypoint_manager = KeypointManager(**keypoint_args)

    def close(self):
        self.joint_state_subscriber.destroy_node()
        self.force_subscriber.destroy_node()
        self.command_publisher.destroy_node()
        rclpy.shutdown()

    def position_control_loop(self, action, z_offset=0.0):
        goal_joints = np.array([100, 100, 100, 100, 100, 100])
        while np.linalg.norm(goal_joints - self.last_joint_pos) > 0.01:
            self._get_latest_observation()
            goal_joints = self.command_publisher.send_action(
                action=action, last_joint_pos=self.last_joint_pos, z_offset=z_offset
            )
        self.command_publisher.send_zeros()

    def reset(self):
        self._get_latest_observation()

        self.position_control_loop(
            action=self.base_command, last_joint_pos=self.reset_joint_pos
        )

        if self.keypoints_in_obs:
            self.keypoint_manager.reset()

        obs, info = self._get_latest_observation()
        return obs, info

    def hard_reset(self):
        self._get_latest_observation()

        self.position_control_loop(
            action=self.base_command,
            z_offset=0.15,
        )

        self.position_control_loop(
            action=self.base_command,
        )

        if self.keypoints_in_obs:
            self.keypoint_manager.reset()

        obs, info = self._get_latest_observation()
        return obs, info

    def step(self, action, bypass_acc_limit):
        if self.last_joint_pos is None:
            self._get_latest_observation()
        action = self._limit_action(action)
        self.command_publisher.send_action(
            action=action,
            last_joint_pos=self.last_joint_pos,
            bypass_acc_limit=bypass_acc_limit,
        )
        obs, info = self._get_latest_observation()

        done = self._get_done(obs)
        reward = -1 if not done else 1500

        return obs, reward, done, False, info

    def _get_done(self, obs):
        done = np.linalg.norm(obs[:2] - np.array([1.2, 0])) < 0.05
        return done

    def seed(self, seed=None):
        pass

    def _get_latest_observation(self):
        self.joint_state_subscriber.last_obs = None
        while self.joint_state_subscriber.last_obs is None:
            rclpy.spin_once(self.joint_state_subscriber)
        obs = self.joint_state_subscriber.last_obs

        if self.use_force_obs:
            self.force_subscriber.last_obs = None
            while self.force_subscriber.last_obs is None:
                rclpy.spin_once(self.force_subscriber)
            obs = np.concatenate((obs, self.force_subscriber.last_obs))

        if self.keypoints_in_obs:
            kp_added = self.keypoint_manager.add_keypoint(
                self.joint_state_subscriber.last_obs,
                self.force_subscriber.last_obs,
            )
            if kp_added:
                print(np.round(self.keypoint_manager.all_keypoints[0], 2))

            for kp in range(self.keypoint_manager.num_keypoints):
                obs = np.concatenate(
                    (
                        obs,
                        self.keypoint_manager.all_keypoints[kp][0],
                        self.keypoint_manager.all_keypoints[kp][1],
                    )
                )

        self.last_joint_pos = self.joint_state_subscriber.last_joint_pos
        self.last_obs = obs
        return obs, {}

    def _limit_action(self, action):
        shortened_action = np.clip(
            action, self.last_obs[:2] - 0.02, self.last_obs[:2] + 0.02
        )
        return shortened_action
