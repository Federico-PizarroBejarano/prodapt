import numpy as np
import shutil, os
import pandas as pd
import zarr

from prodapt.dataset.bag_file_parser import BagFileParser


def process_trajectory(traj_name):
    bag_file = f"/home/eels/prodapt/data/ur10/{traj_name}/{traj_name}_0.db3"
    parser = BagFileParser(bag_file)

    data_js = parser.get_messages("/joint_states")
    data_ur = parser.get_messages("/urscript_interface/script_command")
    df_js = build_dataframe(data_js, mode="joint_states")
    df_ur = build_dataframe(data_ur, mode="urscript")

    df = pd.merge_asof(df_ur, df_js, on="timestamp", direction="nearest")

    return df


def build_dataframe(data, mode):
    if mode == "urscript":
        for i in range(len(data)):
            message = data[i][1].data
            commands = message.split("movel(p[")[1].split("]")[0]
            commands = [float(cmd) for cmd in commands.split(", ")]
            datum = [data[i][0]] + commands
            data[i] = datum
        df = pd.DataFrame(
            data,
            columns=["timestamp", "x", "y", "z", "rx", "ry", "rz"],
            dtype=np.float64,
        )
    elif mode == "joint_states":
        ordered_link_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        for i in range(len(data)):
            message = data[i][1]
            link_names = message.name
            reorder = [link_names.index(name) for name in ordered_link_names]
            positions = list(np.array(message.position)[reorder])
            velocities = list(np.array(message.velocity)[reorder])
            efforts = list(np.array(message.effort)[reorder])
            datum = [data[i][0]] + positions + velocities + efforts
            data[i] = datum
        df = pd.DataFrame(
            data,
            columns=[
                "timestamp",
                "x1",
                "x2",
                "x3",
                "x4",
                "x5",
                "x6",
                "v1",
                "v2",
                "v3",
                "v4",
                "v5",
                "v6",
                "e1",
                "e2",
                "e3",
                "e4",
                "e5",
                "e6",
            ],
            dtype=np.float64,
        )

    return df


def build_dataset():
    path = "/home/eels/prodapt/data/ur10/"

    shutil.rmtree(path + "ur10.zarr", ignore_errors=True)
    f = zarr.group(path + "ur10.zarr")
    dataset = f.create_group("data")
    dataset_started = False

    meta = f.create_group("meta")
    episode_ends = []

    directories = [
        d for d in os.listdir(path) if os.path.isdir(path + d) and "traj" in d
    ]
    for traj_name in directories:
        df = process_trajectory(traj_name)

        ep_actions = df[["x", "y", "z", "rx", "ry", "rz"]].astype(np.float32).to_numpy()
        ep_states = (
            df[
                [
                    "x1",
                    "x2",
                    "x3",
                    "x4",
                    "x5",
                    "x6",
                    "v1",
                    "v2",
                    "v3",
                    "v4",
                    "v5",
                    "v6",
                    "e1",
                    "e2",
                    "e3",
                    "e4",
                    "e5",
                    "e6",
                ]
            ]
            .astype(np.float32)
            .to_numpy()
        )

        if not dataset_started:
            actions = dataset.create_dataset("action", data=ep_actions)
            states = dataset.create_dataset("state", data=ep_states)
            episode_ends.append(len(ep_actions))
            dataset_started = True
        else:
            actions.append(ep_actions)
            states.append(ep_states)
            episode_ends.append(episode_ends[-1] + len(ep_actions))

    meta.create_dataset("episode_ends", data=episode_ends)


if __name__ == "__main__":
    build_dataset()
