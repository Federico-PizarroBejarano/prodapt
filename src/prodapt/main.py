import hydra
from omegaconf import DictConfig

from prodapt.diffusion_policy import DiffusionPolicy, create_push_t_dataloader

# HACK: fix these relative paths for configs
@hydra.main(version_base=None, config_path="../../config", config_name="config")
def main_app(cfg: DictConfig) -> None:
    obs_dim = 5
    action_dim = 2

    # load data
    dataset_path = "./data/push_t_data.zarr"
    # parameters
    pred_horizon = 16
    obs_horizon = 2
    action_horizon = 8
    # |o|o|                             observations: 2
    # | |a|a|a|a|a|a|a|a|               actions executed: 8
    # |p|p|p|p|p|p|p|p|p|p|p|p|p|p|p|p| actions predicted: 16

    # create dataset from file
    dataloader, stats = create_push_t_dataloader(
        dataset_path, pred_horizon, obs_horizon, action_horizon
    )

    diffusion_policy = DiffusionPolicy(
        obs_dim,
        action_dim,
        obs_horizon,
        pred_horizon,
        action_horizon,
        stats,
        num_diffusion_iters=100,
    )

    if cfg.main.mode == "train":
        num_epochs = 100
        diffusion_policy.train(num_epochs, dataloader)
    elif cfg.main.mode == "inference":
        diffusion_policy.load(cfg.main.checkpoint)
        diffusion_policy.inference()


if __name__ == "__main__":
    main_app()
