import numpy as np

from prodapt.utils.rotation_utils import real_exp_transform


class KeypointManager:
    def __init__(self, num_keypoints, min_dist, threshold_force):
        self.num_keypoints = num_keypoints
        self.min_dist = min_dist
        self.threshold_force = threshold_force
        self.torque_ids = [3,4]
        self.reset()

    def add_keypoint(self, position, force_torque):
        force_torque[self.torque_ids] = real_exp_transform(force_torque[self.torque_ids], inverse=True)
        if not self._detect_contact(force_torque[self.torque_ids]):
            return False

        for keypoint in self.all_keypoints:
            dist = np.linalg.norm(np.squeeze(position) - np.squeeze(keypoint[0]))
            if dist <= self.min_dist:
                angle_2rep = np.squeeze(self._get_yaw(force_torque[self.torque_ids]))
                angle_dist = np.linalg.norm(np.squeeze(keypoint[1]) - angle_2rep)
                if angle_dist < 0.75:
                    return False

        self._queue_keypoint(position, force_torque[self.torque_ids])
        return True

    def _queue_keypoint(self, position, force_torque):
        print('ADDING KEYPOINT!!', np.round(position,2), np.round(force_torque,2))
        self.all_keypoints[1:] = self.all_keypoints[:-1]

        angle_2rep = self._get_yaw(force_torque)
        self.all_keypoints[0] = (position, angle_2rep)

    def _detect_contact(self, force_torque):
        return np.linalg.norm(force_torque) > self.threshold_force

    def _get_yaw(self, force_torque):
        angle = np.arctan2(force_torque[1], force_torque[0])
        angle_2rep = [np.sin(angle), np.cos(angle)]
        return angle_2rep

    def reset(self):
        self.all_keypoints = [
            (np.zeros((2)), np.zeros(2)) for i in range(self.num_keypoints)
        ]
