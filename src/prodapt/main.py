import hydra
from omegaconf import DictConfig, OmegaConf

from prodapt.dataset.state_dataset import create_state_dataloader
from prodapt.diffusion_policy import DiffusionPolicy
from prodapt.envs.push_t_env import PushTEnv
from prodapt.envs.ur10_env import UR10Env


# HACK: Fix these relative paths for configs
@hydra.main(version_base=None, config_path="../../config")
def main_app(cfg: DictConfig) -> None:
    # Create dataloader
    if cfg.keypoints_in_obs:
        keypoint_obs = [f"keypoint{i}" for i in range(cfg.keypoint_args.num_keypoints)]
    else:
        keypoint_obs = []

    dataloader, stats, action_dim, obs_dim = create_state_dataloader(
        dataset_path=cfg.train.dataset_path,
        action_list=cfg.action_list,
        obs_list=cfg.obs_list + keypoint_obs,
        pred_horizon=cfg.parameters.pred_horizon,
        obs_horizon=cfg.parameters.obs_horizon,
        action_horizon=cfg.parameters.action_horizon,
    )

    if cfg.name == "push_t":
        env = PushTEnv()
    elif cfg.name == "ur10":
        env = UR10Env(
            controller=cfg.controller,
            action_list=cfg.action_list,
            obs_list=cfg.obs_list + keypoint_obs,
            simulator=cfg.inference.simulator,
            keypoint_args=None if not cfg.keypoints_in_obs else cfg.keypoint_args,
        )
    else:
        raise NotImplementedError(f"Unknown environment type ({cfg.name}).")

    diffusion_policy = DiffusionPolicy(
        env=env,
        obs_dim=obs_dim,
        action_dim=action_dim,
        obs_horizon=cfg.parameters.obs_horizon,
        pred_horizon=cfg.parameters.pred_horizon,
        action_horizon=cfg.parameters.action_horizon,
        training_data_stats=stats,
        num_diffusion_iters=cfg.parameters.num_diffusion_iters,
        seed=cfg.seed,
        use_transformer=cfg.use_transformer,
        network_args=cfg.transformer_args if cfg.use_transformer else cfg.unet_args,
    )

    if cfg.mode == "train":
        OmegaConf.save(cfg, cfg.inference.checkpoint_path.replace(".pt", ".yaml"))
        diffusion_policy.train(
            num_epochs=cfg.train.num_epochs,
            dataloader=dataloader,
            checkpoint_path=cfg.inference.checkpoint_path,
        )
    elif cfg.mode == "inference":
        diffusion_policy.load(input_path=cfg.inference.checkpoint_path)
        diffusion_policy.evaluate(
            num_inferences=1,
            max_steps=cfg.inference.max_steps,
            render=cfg.inference.render,
            warmstart=cfg.inference.warmstart,
        )
    elif cfg.mode == "evaluate":
        diffusion_policy.load(input_path=cfg.inference.checkpoint_path)
        diffusion_policy.evaluate(
            num_inferences=cfg.inference.num_inferences,
            max_steps=cfg.inference.max_steps,
            render=cfg.inference.render,
            warmstart=cfg.inference.warmstart,
        )


if __name__ == "__main__":
    main_app()
