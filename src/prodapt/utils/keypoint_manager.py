import numpy as np


class KeypointManager:
    def __init__(self, num_keypoints, min_dist, threshold_force):
        self.num_keypoints = num_keypoints
        self.min_dist = min_dist
        self.threshold_force = threshold_force
        self.reset()

    def add_keypoint(self, abs_time, position, torque2):
        assert len(torque2) == 2
        if not self._detect_contact(torque2):
            return False

        for keypoint in self.all_keypoints:
            dist = np.linalg.norm(np.squeeze(position) - np.squeeze(keypoint[1]))
            if dist <= self.min_dist:
                angle_2rep = np.squeeze(self._get_yaw(torque2))
                angle_dist = np.linalg.norm(np.squeeze(keypoint[2]) - angle_2rep)
                if angle_dist < 0.75:
                    return False

        self._queue_keypoint(abs_time, position, torque2)
        return True

    def _queue_keypoint(self, abs_time, position, torque2):
        self.all_keypoints[1:] = self.all_keypoints[:-1]

        angle_2rep = self._get_yaw(torque2)
        self.all_keypoints[0] = ([abs_time], position, angle_2rep)

    def _detect_contact(self, torque2):
        return np.linalg.norm(torque2) > self.threshold_force

    def _get_yaw(self, ee_torque):
        angle = np.arctan2(ee_torque[1], ee_torque[0])
        angle_2rep = (np.sin(angle), np.cos(angle))
        return angle_2rep

    def reset(self):
        self.all_keypoints = [
            ([0], np.zeros((2)), np.zeros(2)) for i in range(self.num_keypoints)
        ]
