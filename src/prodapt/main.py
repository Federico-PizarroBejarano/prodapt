import hydra
from omegaconf import DictConfig

from prodapt.dataset.state_dataset import create_state_dataloader
from prodapt.diffusion_policy import DiffusionPolicy
from prodapt.envs.push_t_env import PushTEnv


# HACK: Fix these relative paths for configs
@hydra.main(version_base=None, config_path="../../config")
def main_app(cfg: DictConfig) -> None:
    # Create dataloader
    dataloader, stats = create_state_dataloader(
        dataset_path=cfg.train.dataset_path,
        pred_horizon=cfg.parameters.pred_horizon,
        obs_horizon=cfg.parameters.obs_horizon,
        action_horizon=cfg.parameters.action_horizon,
    )

    if cfg.name == "push_t":
        env_func = PushTEnv
    else:
        raise NotImplementedError(f"Unknown environment type ({cfg.name}).")

    diffusion_policy = DiffusionPolicy(
        env_func=env_func,
        obs_dim=cfg.parameters.obs_dim,
        action_dim=cfg.parameters.action_dim,
        obs_horizon=cfg.parameters.obs_horizon,
        pred_horizon=cfg.parameters.pred_horizon,
        action_horizon=cfg.parameters.action_horizon,
        training_data_stats=stats,
        num_diffusion_iters=cfg.parameters.num_diffusion_iters,
        seed=cfg.seed,
    )

    if cfg.mode == "train":
        diffusion_policy.train(num_epochs=cfg.train.num_epochs, dataloader=dataloader)
    elif cfg.mode == "inference":
        diffusion_policy.load(input_path=cfg.inference.checkpoint_path)
        diffusion_policy.inference(
            max_steps=cfg.inference.max_steps, render=cfg.inference.render
        )


if __name__ == "__main__":
    main_app()
