"""StateDataset class

Defines generic StateDataset class

The StateDataset class
- Loads data (obs, action) from a zarr storage
- Normalizes each dimension of obs and action to [-1,1]
- Returns
  - All possible segments with length `pred_horizon`
  - Pads the beginning and the end of each episode with repetition
  - key `obs`: shape (obs_horizon, obs_dim)
  - key `action`: shape (pred_horizon, action_dim)
"""

import torch
import zarr

import numpy as np

from prodapt.dataset.dataset_utils import (
    create_sample_indices,
    get_data_stats,
    sample_sequence,
    normalize_data,
)


class StateDataset(torch.utils.data.Dataset):
    def __init__(
        self,
        dataset_path,
        action_list,
        obs_list,
        pred_horizon,
        obs_horizon,
        action_horizon,
    ):
        # Read from zarr dataset
        dataset_root = zarr.open(dataset_path, "r")

        # All demonstration episodes are concatenated in the first dimension N
        actions = np.hstack(
            [dataset_root["data"]["action"][key][:] for key in action_list]
        )
        self.action_dim = actions.shape[1]
        obs = np.hstack([dataset_root["data"]["obs"][key][:] for key in obs_list])
        self.obs_dim = obs.shape[1]
        self.real_obs_dim = np.hstack(
            [
                dataset_root["data"]["obs"][key][:]
                for key in obs_list
                if "keypoint" not in key
            ]
        ).shape[1]

        train_data = {"action": actions, "obs": obs}  # (N, action_dim)  # (N, obs_dim)

        # Marks one-past the last index for each episode
        episode_ends = dataset_root["meta"]["episode_ends"][:]
        print("meta data: ", dataset_root.tree())

        # Computes start and end of each state-action sequence and pads
        indices = create_sample_indices(
            episode_ends=episode_ends,
            sequence_length=pred_horizon,
            # Add padding such that each timestep in the dataset is seen
            pad_before=obs_horizon - 1,
            pad_after=action_horizon - 1,
        )

        # Compute statistics and normalize data to [-1,1]
        stats = dict()
        normalized_train_data = dict()
        for key, data in train_data.items():
            stats[key] = get_data_stats(data, key, obs_list, self.real_obs_dim)
            normalized_train_data[key] = normalize_data(data, stats[key])

        self.indices = indices
        self.stats = stats
        self.normalized_train_data = normalized_train_data
        self.pred_horizon = pred_horizon
        self.action_horizon = action_horizon
        self.obs_horizon = obs_horizon

    def __len__(self):
        # All possible segments of the dataset
        return len(self.indices)

    def __getitem__(self, idx):
        # Get the start/end indices for this datapoint
        (
            buffer_start_idx,
            buffer_end_idx,
            sample_start_idx,
            sample_end_idx,
        ) = self.indices[idx]

        # Get normalized data using these indices
        nsample = sample_sequence(
            train_data=self.normalized_train_data,
            sequence_length=self.pred_horizon,
            buffer_start_idx=buffer_start_idx,
            buffer_end_idx=buffer_end_idx,
            sample_start_idx=sample_start_idx,
            sample_end_idx=sample_end_idx,
        )

        # Discard unused observations
        nsample["obs"] = nsample["obs"][: self.obs_horizon, :]
        return nsample


def create_state_dataloader(
    dataset_path, action_list, obs_list, pred_horizon, obs_horizon, action_horizon
):
    # Create dataset from file
    dataset = StateDataset(
        dataset_path=dataset_path,
        action_list=action_list,
        obs_list=obs_list,
        pred_horizon=pred_horizon,
        obs_horizon=obs_horizon,
        action_horizon=action_horizon,
    )
    # Save training data statistics (min, max) for each dim
    stats = dataset.stats

    # Create DataLoader
    dataloader = torch.utils.data.DataLoader(
        dataset,
        batch_size=256,
        num_workers=1,
        shuffle=True,
        # Accelerate cpu-gpu transfer
        pin_memory=True,
        # Don't kill worker process afte each epoch
        persistent_workers=True,
    )
    return dataloader, stats, dataset.action_dim, dataset.obs_dim, dataset.real_obs_dim


if __name__ == "__main__":
    dataset_path = "./data/push_t_data.zarr"

    pred_horizon = 16
    obs_horizon = 2
    action_horizon = 8
    # |o|o|                              observations: 2
    # | |a|a|a|a|a|a|a|a|                actions executed: 8
    # |p|p|p|p|p|p|p|p|p|p|p|p|p|p|p|p|  actions predicted: 16

    action_list = ["ee_pose"]  # possible values: ee_pose
    obs_list = ["state"]  # possible values: state, img, keypoint, n_contacts

    dataloader, stats, action_dim, obs_dim, real_obs_dim = create_state_dataloader(
        dataset_path, action_list, obs_list, pred_horizon, obs_horizon, action_horizon
    )

    # Visualize data in batch
    batch = next(iter(dataloader))
    print("batch['obs'].shape:", batch["obs"].shape)
    print("batch['action'].shape", batch["action"].shape)
    print("batch['obs']:", batch["obs"][0, 0, :])
    print("batch['action']", batch["action"][0, 0, :])
