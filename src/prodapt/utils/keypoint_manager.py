import numpy as np


class KeypointManager:
    def __init__(self, num_keypoints, min_dist, threshold_force):
        self.num_keypoints = num_keypoints
        self.min_dist = min_dist
        self.threshold_force = threshold_force
        self.reset()

    def add_keypoint(self, position, force_torque):
        if not self._detect_contact(force_torque):
            return False

        for keypoint in self.all_keypoints:
            dist = np.linalg.norm(np.squeeze(position) - np.squeeze(keypoint[0]))
            if dist <= self.min_dist:
                angle_2rep = np.squeeze(self._get_yaw(force_torque))
                angle_dist = np.linalg.norm(np.squeeze(keypoint[1]) - angle_2rep)
                if angle_dist < 0.75:
                    return False

        self._queue_keypoint(position, force_torque)
        return True

    def _queue_keypoint(self, position, force_torque):
        self.all_keypoints[1:] = self.all_keypoints[:-1]

        angle_2rep = self._get_yaw(force_torque)
        self.all_keypoints[0] = (position, angle_2rep)

    def _detect_contact(self, force_torque):
        torque = force_torque[4:]
        return np.linalg.norm(torque) > self.threshold_force

    def _get_yaw(self, force_torque):
        angle = np.arctan2(force_torque[5], force_torque[4])
        angle_2rep = (np.sin(angle), np.cos(angle))
        return angle_2rep

    def reset(self):
        self.all_keypoints = [
            (np.zeros((2)), np.zeros(2)) for i in range(self.num_keypoints)
        ]
