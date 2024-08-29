import numpy as np
import shutil
import pandas as pd
import zarr

import matplotlib.pyplot as plt

from prodapt.utils.keypoint_manager import KeypointManager
from prodapt.dataset.bag_file_parser import BagFileParser
from prodapt.utils.kinematics_utils import forward_kinematics
from prodapt.utils.rotation_utils import matrix_to_rotation_6d

ordered_link_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

action_keys = [
    "commanded_joint_pos",
    "commanded_ee_position",
    "commanded_ee_position_xy",
    "commanded_ee_rotation_6d",
]
obs_keys = [
    "joint_pos",
    "joint_vel",
    "joint_eff",
    "ee_position",
    "ee_position_xy",
    "ee_rotation_6d",
    "force",
    "torque",
    "torque2",
    "torque_angle",
]


def rosbag_to_dataframe(rosbag_name, plot=False):
    bag_file = f"/home/{USER}/prodapt/data/ur10/{rosbag_name}/{rosbag_name}_0.db3"
    parser = BagFileParser(bag_file)

    data_joints = parser.get_messages("/joint_states")
    data_commands = parser.get_messages("/joint_command")
    # May also be /force_torque_sensor_broadcaster/wrench
    data_forces = parser.get_messages("/force_torque")

    df_joints = build_dataframe(data_joints, mode="joint_states")
    df_commands = build_dataframe(data_commands, mode="joint_command")
    df_forces = build_dataframe(data_forces, mode="force_torque")

    df_joints.columns = ["__".join(a) for a in df_joints.columns.to_flat_index()]
    df_commands.columns = ["__".join(a) for a in df_commands.columns.to_flat_index()]
    df_forces.columns = ["__".join(a) for a in df_forces.columns.to_flat_index()]

    df = pd.merge_asof(
        df_commands, df_joints, on="timestamp__timestamp", direction="nearest"
    )
    df = pd.merge_asof(df, df_forces, on="timestamp__timestamp", direction="nearest")

    df.columns = pd.MultiIndex.from_tuples([a.split("__") for a in df.columns])

    episode_ends = get_episode_ends(df["joint_pos"])

    if plot is True:
        episode_ends = [0] + episode_ends
        for ep in range(len(episode_ends) - 1):
            positions = np.array(df["ee_position"][["x", "y"]])[
                episode_ends[ep] : episode_ends[ep + 1]
            ]
            plt.plot(positions[:, 0], positions[:, 1])
            plt.xlim(0.3, 1.3)
            plt.ylim(-0.45, 0.45)
            plt.show()
        episode_ends = episode_ends[1:]

    return df[: episode_ends[-1]], episode_ends


def get_episode_ends(all_joint_pos):
    i = 100
    episode_ends = []
    all_dists = np.linalg.norm(
        np.array(all_joint_pos)[1:] - np.array(all_joint_pos)[:-1], axis=1
    )
    while i < len(all_dists):
        idx = np.argmax(all_dists[i:] > 0.7)
        if idx == 0:
            break
        ee_end = i + idx + 1
        episode_ends.append(ee_end)
        i = ee_end + 100

    return episode_ends


def build_dataframe(data, mode):
    if mode == "joint_states":
        all_data = {
            "timestamp": [],
            "joint_pos": [],
            "joint_vel": [],
            "joint_eff": [],
            "ee_position": [],
            "ee_position_xy": [],
            "ee_rotation_6d": [],
        }
        for i in range(len(data)):
            message = data[i][1]
            link_names = message.name
            reorder = [link_names.index(name) for name in ordered_link_names]
            all_data["timestamp"].append(data[i][0])
            all_data["joint_pos"].append(np.array(message.position)[reorder])
            all_data["joint_vel"].append(np.array(message.velocity)[reorder])
            all_data["joint_eff"].append(np.array(message.effort)[reorder])
            T_matrix = forward_kinematics(
                np.array(message.position)[reorder].reshape(6, 1)
            )
            translation = T_matrix[:3, 3].squeeze()
            rotation_6d = matrix_to_rotation_6d(T_matrix[:3, :3]).squeeze()
            all_data["ee_position"].append(translation)
            all_data["ee_position_xy"].append(translation[:2])
            all_data["ee_rotation_6d"].append(rotation_6d)

        df = {}
        df["timestamp"] = pd.DataFrame(all_data["timestamp"], columns=["timestamp"])
        df["joint_pos"] = pd.DataFrame(
            all_data["joint_pos"], columns=["x1", "x2", "x3", "x4", "x5", "x6"]
        )
        df["joint_vel"] = pd.DataFrame(
            all_data["joint_vel"], columns=["v1", "v2", "v3", "v4", "v5", "v6"]
        )
        df["joint_eff"] = pd.DataFrame(
            all_data["joint_eff"], columns=["e1", "e2", "e3", "e4", "e5", "e6"]
        )
        df["ee_position"] = pd.DataFrame(
            all_data["ee_position"], columns=["x", "y", "z"]
        )
        df["ee_position_xy"] = pd.DataFrame(
            all_data["ee_position_xy"], columns=["x", "y"]
        )
        df["ee_rotation_6d"] = pd.DataFrame(
            all_data["ee_rotation_6d"], columns=["a1", "a2", "a3", "b1", "b2", "b3"]
        )

        df = pd.concat(df, axis=1).astype(np.float64)
    elif mode == "joint_command":
        all_data = {
            "timestamp": [],
            "commanded_joint_pos": [],
            "commanded_ee_position": [],
            "commanded_ee_position_xy": [],
            "commanded_ee_rotation_6d": [],
        }
        for i in range(len(data)):
            message = data[i][1]
            link_names = message.name
            reorder = [link_names.index(name) for name in ordered_link_names]
            all_data["timestamp"].append(data[i][0])
            all_data["commanded_joint_pos"].append(np.array(message.position)[reorder])
            T_matrix = forward_kinematics(
                np.array(message.position)[reorder].reshape(6, 1)
            )
            translation = T_matrix[:3, 3].squeeze()
            rotation_6d = matrix_to_rotation_6d(T_matrix[:3, :3]).squeeze()
            all_data["commanded_ee_position"].append(translation)
            all_data["commanded_ee_position_xy"].append(translation[:2])
            all_data["commanded_ee_rotation_6d"].append(rotation_6d)

        df = {}
        df["timestamp"] = pd.DataFrame(all_data["timestamp"], columns=["timestamp"])
        df["commanded_joint_pos"] = pd.DataFrame(
            all_data["commanded_joint_pos"],
            columns=["x1", "x2", "x3", "x4", "x5", "x6"],
        )
        df["commanded_ee_position"] = pd.DataFrame(
            all_data["commanded_ee_position"], columns=["x", "y", "z"]
        )
        df["commanded_ee_position_xy"] = pd.DataFrame(
            all_data["commanded_ee_position_xy"], columns=["x", "y"]
        )
        df["commanded_ee_rotation_6d"] = pd.DataFrame(
            all_data["commanded_ee_rotation_6d"],
            columns=["a1", "a2", "a3", "b1", "b2", "b3"],
        )

        df = pd.concat(df, axis=1).astype(np.float64)
    elif mode == "force_torque":
        all_data = {
            "timestamp": [],
            "force": [],
            "torque": [],
            "torque2": [],
            "torque_angle": [],
        }
        for i in range(len(data)):
            message = data[i][1]
            all_data["timestamp"].append(data[i][0])
            all_data["force"].append(
                np.array(
                    [
                        message.wrench.force.x,
                        message.wrench.force.y,
                        message.wrench.force.z,
                    ]
                )
            )
            all_data["torque"].append(
                np.array(
                    [
                        message.wrench.torque.x,
                        message.wrench.torque.y,
                        message.wrench.torque.z,
                    ]
                )
            )
            all_data["torque2"].append(
                np.array([message.wrench.torque.y, message.wrench.torque.z])
            )
            if keypoint_manager._detect_contact(all_data["torque2"][-1]):
                angle_rep = keypoint_manager._get_yaw(all_data["torque2"][-1])
            else:
                angle_rep = np.array([0.0, 0.0])
            all_data["torque_angle"].append(angle_rep)

        df = {}
        df["timestamp"] = pd.DataFrame(all_data["timestamp"], columns=["timestamp"])
        df["force"] = pd.DataFrame(all_data["force"], columns=["x", "y", "z"])
        df["torque"] = pd.DataFrame(all_data["torque"], columns=["x", "y", "z"])
        df["torque2"] = pd.DataFrame(all_data["torque2"], columns=["y", "z"])
        df["torque_angle"] = pd.DataFrame(
            all_data["torque_angle"], columns=["sin_yaw", "cos_yaw"]
        )

        df = pd.concat(df, axis=1).astype(np.float64)

    return df


def build_dataset(new_dataset_name, rosbag_names, keypoint_args):
    path = f"/home/{USER}/prodapt/data/ur10/"

    shutil.rmtree(path + f"{new_dataset_name}.zarr", ignore_errors=True)
    f = zarr.group(path + f"{new_dataset_name}.zarr")
    dataset = f.create_group("data")

    actions = dataset.create_group("action")
    obs = dataset.create_group("obs")

    dfs = []
    episode_ends_lists = np.array([0])

    for rosbag in rosbag_names:
        df, episode_ends = rosbag_to_dataframe(rosbag)
        df = add_keypoints(df, episode_ends, keypoint_args)
        dfs.append(df)
        episode_ends_lists = np.concatenate(
            [episode_ends_lists, np.squeeze(episode_ends) + episode_ends_lists[-1]]
        )

    df = pd.concat(dfs, ignore_index=True)
    episode_ends = episode_ends_lists[1:]

    for key in action_keys:
        actions.create_dataset(key, data=df[key].astype(np.float32).to_numpy())

    for key in obs_keys:
        obs.create_dataset(key, data=df[key].astype(np.float32).to_numpy())

    for kp in range(keypoint_args["num_keypoints"]):
        obs.create_dataset(
            f"keypoint{kp}", data=df[f"keypoint{kp}"].astype(np.float32).to_numpy()
        )

    meta = f.create_group("meta")
    meta.create_dataset("episode_ends", data=episode_ends)


def add_keypoints(df, episode_ends, keypoint_args):
    kp_headers = [f"keypoint{i}" for i in range(keypoint_args["num_keypoints"])]
    kp_subheaders = ["x", "y", "sin_yaw", "cos_yaw"]
    midx = pd.MultiIndex.from_product([kp_headers, kp_subheaders])

    keypoints_df = pd.DataFrame(index=range(df.shape[0]), columns=midx)
    new_df = pd.concat([df, keypoints_df], axis=1)
    new_df.loc[:, kp_headers] = 0.0

    aug_episode_ends = [0] + episode_ends
    for ee in range(1, len(aug_episode_ends)):
        keypoint_manager.reset()
        num_keypoints = 0

        for idx in range(aug_episode_ends[ee - 1], aug_episode_ends[ee]):
            position = list(new_df.loc[idx, "ee_position_xy"])
            torque2 = list(new_df.loc[idx, "torque2"])

            added = keypoint_manager.add_keypoint(position, torque2)
            for kp in range(keypoint_args["num_keypoints"]):
                new_df.loc[idx, f"keypoint{kp}"] = np.concatenate(
                    keypoint_manager.all_keypoints[kp]
                )
            if added:
                num_keypoints += 1

        print(ee, num_keypoints)

    return new_df


if __name__ == "__main__":
    USER = "eels"
    rosbag_names = ["cube", "cube2"]
    new_dataset_name = "cube"
    keypoint_args = {"num_keypoints": 15, "min_dist": 0.05, "threshold_force": 1.0}
    keypoint_manager = KeypointManager(**keypoint_args)
    build_dataset(new_dataset_name, rosbag_names, keypoint_args)
