import numpy as np
import shutil
import pandas as pd
import zarr

import matplotlib.pyplot as plt

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
    "commanded_ee_rotation_6d",
]
obs_keys = [
    "joint_pos",
    "joint_vel",
    "joint_eff",
    "ee_position",
    "ee_rotation_6d",
    "force",
    "torque",
]


def process_trajectory(plot=False):
    bag_file = f"/home/eels/prodapt/data/ur10/cube/cube_0.db3"
    parser = BagFileParser(bag_file)

    data_joints = parser.get_messages("/joint_states")
    data_commands = parser.get_messages("/joint_command")
    data_forces = parser.get_messages("/force_torque")

    df_joints = build_dataframe(data_joints, mode="joint_states")
    df_commands = build_dataframe(data_commands, mode="joint_command")
    df_forces = build_dataframe(data_forces, mode="force_torque")

    # TODO: Find a better solution for merging two multi-column Dataframes
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
            plt.show()
        episode_ends = episode_ends[1:]

    return df, episode_ends


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
            all_data["ee_rotation_6d"].append(rotation_6d)

        df = {}
        df["timestamp"] = pd.DataFrame(
            all_data["timestamp"],
            columns=["timestamp"],
        )
        df["joint_pos"] = pd.DataFrame(
            all_data["joint_pos"],
            columns=["x1", "x2", "x3", "x4", "x5", "x6"],
        )
        df["joint_vel"] = pd.DataFrame(
            all_data["joint_vel"],
            columns=["v1", "v2", "v3", "v4", "v5", "v6"],
        )
        df["joint_eff"] = pd.DataFrame(
            all_data["joint_eff"],
            columns=["e1", "e2", "e3", "e4", "e5", "e6"],
        )
        df["ee_position"] = pd.DataFrame(
            all_data["ee_position"],
            columns=["x", "y", "z"],
        )
        df["ee_rotation_6d"] = pd.DataFrame(
            all_data["ee_rotation_6d"],
            columns=["a1", "a2", "a3", "b1", "b2", "b3"],
        )

        df = pd.concat(df, axis=1).astype(np.float64)
    elif mode == "joint_command":
        all_data = {
            "timestamp": [],
            "commanded_joint_pos": [],
            "commanded_ee_position": [],
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
            all_data["commanded_ee_rotation_6d"].append(rotation_6d)

        df = {}
        df["timestamp"] = pd.DataFrame(
            all_data["timestamp"],
            columns=["timestamp"],
        )
        df["commanded_joint_pos"] = pd.DataFrame(
            all_data["commanded_joint_pos"],
            columns=["x1", "x2", "x3", "x4", "x5", "x6"],
        )
        df["commanded_ee_position"] = pd.DataFrame(
            all_data["commanded_ee_position"],
            columns=["x", "y", "z"],
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

        df = {}
        df["timestamp"] = pd.DataFrame(
            all_data["timestamp"],
            columns=["timestamp"],
        )
        df["force"] = pd.DataFrame(
            all_data["force"],
            columns=["x", "y", "z"],
        )
        df["torque"] = pd.DataFrame(
            all_data["torque"],
            columns=["x", "y", "z"],
        )

        df = pd.concat(df, axis=1).astype(np.float64)

    return df


def build_dataset():
    path = "/home/eels/prodapt/data/ur10/"

    shutil.rmtree(path + "cube.zarr", ignore_errors=True)
    f = zarr.group(path + "cube.zarr")
    dataset = f.create_group("data")

    actions = dataset.create_group("action")
    obs = dataset.create_group("obs")

    df, episode_ends = process_trajectory()

    for key in action_keys:
        actions.create_dataset(key, data=df[key].astype(np.float32).to_numpy())

    for key in obs_keys:
        obs.create_dataset(key, data=df[key].astype(np.float32).to_numpy())

    meta = f.create_group("meta")
    meta.create_dataset("episode_ends", data=episode_ends)


if __name__ == "__main__":
    build_dataset()
    # process_trajectory(plot=True)
