import numpy as np
import shutil, os
import pandas as pd
import zarr

from prodapt.dataset.bag_file_parser import BagFileParser
from prodapt.utils.kinematics_utils import forward_kinematics
from prodapt.utils.rotation_utils import (
    matrix_to_rotation_6d,
    axis_angle_to_rotation_6d,
)

ordered_link_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

action_keys = ["commanded_ee_position", "commanded_ee_rotation_6d"]
obs_keys = ["joint_pos", "joint_vel", "joint_eff", "ee_position", "ee_rotation_6d"]

cwd = os.getcwd()


def process_trajectory(traj_name):
    bag_file = f"{cwd}/data/ur10/trajectories/{traj_name}/{traj_name}_0.db3"
    parser = BagFileParser(bag_file)

    data_joints = parser.get_messages("/joint_states")
    data_urscript = parser.get_messages("/urscript_interface/script_command")

    df_joints = build_dataframe(data_joints, mode="joint_states")
    df_urscript = build_dataframe(data_urscript, mode="urscript")

    df_joints.columns = ["__".join(a) for a in df_joints.columns.to_flat_index()]
    df_urscript.columns = ["__".join(a) for a in df_urscript.columns.to_flat_index()]

    df = pd.merge_asof(
        df_urscript, df_joints, on="timestamp__timestamp", direction="nearest"
    )

    df.columns = pd.MultiIndex.from_tuples([a.split("__") for a in df.columns])

    return df


def build_dataframe(data, mode):
    if mode == "urscript":
        all_data = {key: [] for key in action_keys}
        all_data["timestamp"] = []
        for i in range(len(data)):
            message = data[i][1].data
            commands = message.split("movel(p[")[1].split("]")[0]
            commands = [float(cmd) for cmd in commands.split(", ")]
            all_data["timestamp"].append(data[i][0])
            all_data["commanded_ee_position"].append(commands[:3])
            all_data["commanded_ee_rotation_6d"].append(
                axis_angle_to_rotation_6d(commands[3:])
            )

        df = {}
        df["timestamp"] = pd.DataFrame(all_data["timestamp"], columns=["timestamp"])
        df["commanded_ee_position"] = pd.DataFrame(
            all_data["commanded_ee_position"], columns=["x", "y", "z"]
        )
        df["commanded_ee_rotation_6d"] = pd.DataFrame(
            all_data["commanded_ee_rotation_6d"],
            columns=["a1", "a2", "a3", "b1", "b2", "b3"],
        )

        df = pd.concat(df, axis=1).astype(np.float64)
    elif mode == "joint_states":
        all_data = {key: [] for key in obs_keys}
        all_data["timestamp"] = []
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
        df["ee_rotation_6d"] = pd.DataFrame(
            all_data["ee_rotation_6d"], columns=["a1", "a2", "a3", "b1", "b2", "b3"]
        )

        df = pd.concat(df, axis=1).astype(np.float64)

    return df


def build_dataset():
    path = f"{cwd}/data/ur10/"
    traj_path = path + "heart_trajectories/"

    shutil.rmtree(path + "ur10_heart.zarr", ignore_errors=True)
    f = zarr.group(path + "ur10_heart.zarr")
    dataset = f.create_group("data")

    actions = dataset.create_group("action")
    actions_groups = {}
    obs = dataset.create_group("obs")
    obs_groups = {}
    dataset_started = False

    episode_ends = []

    directories = [d for d in os.listdir(traj_path)]
    for traj_name in directories:
        df = process_trajectory(traj_name)

        if not dataset_started:
            for key in action_keys:
                actions_groups[key] = actions.create_dataset(
                    key, data=df[key].astype(np.float32).to_numpy()
                )

            for key in obs_keys:
                obs_groups[key] = obs.create_dataset(
                    key, data=df[key].astype(np.float32).to_numpy()
                )

            episode_ends.append(len(df[key]))
            dataset_started = True
        else:
            for key in action_keys:
                actions_groups[key].append(df[key].astype(np.float32).to_numpy())

            for key in obs_keys:
                obs_groups[key].append(df[key].astype(np.float32).to_numpy())

            episode_ends.append(episode_ends[-1] + len(df[key]))

    meta = f.create_group("meta")
    meta.create_dataset("episode_ends", data=episode_ends)


if __name__ == "__main__":
    build_dataset()
