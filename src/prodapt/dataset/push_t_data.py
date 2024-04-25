import torch
import zarr

from prodapt.dataset.dataset_utils import (
    create_sample_indices,
    get_data_stats,
    sample_sequence,
    normalize_data,
)


# ### **Dataset**
#
# Defines `PushTStateDataset` and helper functions
#
# The dataset class
# - Load data (obs, action) from a zarr storage
# - Normalizes each dimension of obs and action to [-1,1]
# - Returns
#   - All possible segments with length `pred_horizon`
#   - Pads the beginning and the end of each episode with repetition
#   - key `obs`: shape (obs_horizon, obs_dim)
#   - key `action`: shape (pred_horizon, action_dim)

# dataset
class PushTStateDataset(torch.utils.data.Dataset):
    def __init__(self, dataset_path, pred_horizon, obs_horizon, action_horizon):
        # read from zarr dataset
        dataset_root = zarr.open(dataset_path, "r")
        # All demonstration episodes are concatinated in the first dimension N
        train_data = {
            # (N, action_dim)
            "action": dataset_root["data"]["action"][:],
            # (N, obs_dim)
            "obs": dataset_root["data"]["state"][:],
        }
        # Marks one-past the last index for each episode
        episode_ends = dataset_root["meta"]["episode_ends"][:]
        print("meta data: ", dataset_root.tree())

        # compute start and end of each state-action sequence
        # also handles padding
        indices = create_sample_indices(
            episode_ends=episode_ends,
            sequence_length=pred_horizon,
            # add padding such that each timestep in the dataset are seen
            pad_before=obs_horizon - 1,
            pad_after=action_horizon - 1,
        )

        # compute statistics and normalized data to [-1,1]
        stats = dict()
        normalized_train_data = dict()
        for key, data in train_data.items():
            stats[key] = get_data_stats(data)
            normalized_train_data[key] = normalize_data(data, stats[key])

        self.indices = indices
        self.stats = stats
        self.normalized_train_data = normalized_train_data
        self.pred_horizon = pred_horizon
        self.action_horizon = action_horizon
        self.obs_horizon = obs_horizon

    def __len__(self):
        # all possible segments of the dataset
        return len(self.indices)

    def __getitem__(self, idx):
        # get the start/end indices for this datapoint
        (
            buffer_start_idx,
            buffer_end_idx,
            sample_start_idx,
            sample_end_idx,
        ) = self.indices[idx]

        # get normalized data using these indices
        nsample = sample_sequence(
            train_data=self.normalized_train_data,
            sequence_length=self.pred_horizon,
            buffer_start_idx=buffer_start_idx,
            buffer_end_idx=buffer_end_idx,
            sample_start_idx=sample_start_idx,
            sample_end_idx=sample_end_idx,
        )

        # discard unused observations
        nsample["obs"] = nsample["obs"][: self.obs_horizon, :]
        return nsample


if __name__ == "__main__":
    dataset_path = "./data/push_t_data.zarr"
    # parameters
    pred_horizon = 16
    obs_horizon = 2
    action_horizon = 8
    # |o|o|                             observations: 2
    # | |a|a|a|a|a|a|a|a|               actions executed: 8
    # |p|p|p|p|p|p|p|p|p|p|p|p|p|p|p|p| actions predicted: 16

    # create dataset from file
    dataset = PushTStateDataset(
        dataset_path=dataset_path,
        pred_horizon=pred_horizon,
        obs_horizon=obs_horizon,
        action_horizon=action_horizon,
    )
    # save training data statistics (min, max) for each dim
    stats = dataset.stats

    # create dataloader
    dataloader = torch.utils.data.DataLoader(
        dataset,
        batch_size=256,
        num_workers=1,
        shuffle=True,
        # accelerate cpu-gpu transfer
        pin_memory=True,
        # don't kill worker process afte each epoch
        persistent_workers=True,
    )

    # visualize data in batch
    batch = next(iter(dataloader))
    print("batch['obs'].shape:", batch["obs"].shape)
    print("batch['action'].shape", batch["action"].shape)
    print("batch['obs']:", batch["obs"][0, 0, :])
    print("batch['action']", batch["action"][0, 0, :])
