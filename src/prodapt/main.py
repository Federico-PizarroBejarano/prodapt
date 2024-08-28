import os

import hydra
from omegaconf import DictConfig, OmegaConf

from prodapt.dataset.state_dataset import create_state_dataloader
from prodapt.diffusion_policy import DiffusionPolicy


@hydra.main(version_base=None, config_path="../../config")
def main_app(cfg: DictConfig) -> None:
    print(cfg)

    if cfg.keypoints_in_obs:
        keypoint_obs = [f"keypoint{i}" for i in range(cfg.keypoint_args.num_keypoints)]
    else:
        keypoint_obs = []

    updated_pred_horizon = cfg.parameters.pred_horizon + cfg.parameters.obs_horizon
    if updated_pred_horizon % 4 != 0:
        updated_pred_horizon += 4 - (updated_pred_horizon % 4)

    # Create dataloader
    dataloader, stats, action_dim, obs_dim, real_obs_dim = create_state_dataloader(
        dataset_path=cfg.train.dataset_path,
        action_list=cfg.action_list,
        obs_list=cfg.obs_list + keypoint_obs,
        pred_horizon=updated_pred_horizon,
        obs_horizon=cfg.parameters.obs_horizon,
        action_horizon=cfg.parameters.action_horizon,
    )

    diffusion_policy = DiffusionPolicy(
        obs_dim=obs_dim,
        real_obs_dim=real_obs_dim,
        action_dim=action_dim,
        pred_horizon=updated_pred_horizon,
        obs_horizon=cfg.parameters.obs_horizon,
        action_horizon=cfg.parameters.action_horizon,
        training_data_stats=stats,
        num_diffusion_iters=cfg.parameters.num_diffusion_iters,
        seed=cfg.seed,
        use_transformer=cfg.use_transformer,
        num_keypoints=(
            0 if not cfg.keypoints_in_obs else cfg.keypoint_args.num_keypoints
        ),
        network_args=cfg.transformer_args if cfg.use_transformer else cfg.unet_args,
    )

    if cfg.mode == "train":
        if not os.path.exists(f"./checkpoints/{cfg.model_name}/"):
            os.makedirs(f"./checkpoints/{cfg.model_name}/")
        OmegaConf.save(cfg, f"./checkpoints/{cfg.model_name}/{cfg.model_name}.yaml")
        diffusion_policy.train(
            num_epochs=cfg.train.num_epochs,
            dataloader=dataloader,
            model_name=cfg.model_name,
        )
    else:
        if cfg.name == "push_t":
            from prodapt.envs.push_t_env import PushTEnv

            env = PushTEnv()
        elif cfg.name == "ur10":
            from prodapt.envs.ur10_env import UR10Env

            env = UR10Env(
                controller=cfg.controller,
                action_list=cfg.action_list,
                obs_list=cfg.obs_list + keypoint_obs,
                interface=cfg.inference.interface,
                keypoint_args=None if not cfg.keypoints_in_obs else cfg.keypoint_args,
            )
        else:
            raise NotImplementedError(f"Unknown environment type ({cfg.name}).")

        diffusion_policy.load(
            model_name=cfg.model_name,
            input_path=(
                None
                if ["checkpoint_path"] not in cfg.inference
                else cfg.inference.checkpoint_path
            ),
        )
        diffusion_policy.evaluate(
            env=env,
            num_inferences=(
                1 if cfg.mode == "inference" else cfg.inference.num_inferences
            ),
            max_steps=cfg.inference.max_steps,
            render=cfg.inference.render,
            warmstart=cfg.inference.warmstart,
            model_name=cfg.model_name,
        )


if __name__ == "__main__":
    main_app()
