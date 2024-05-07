import os
import collections
import numpy as np
import torch
import torch.nn as nn
from diffusers.training_utils import EMAModel
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from diffusers.optimization import get_scheduler
from tqdm.auto import tqdm
import hydra
from IPython.display import Video
from skvideo.io import vwrite

from prodapt.dataset.dataset_utils import normalize_data, unnormalize_data
from prodapt.diffusion.conditional_unet_1d import ConditionalUnet1D

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


class DiffusionPolicy:
    def __init__(
        self,
        env,
        obs_dim,
        action_dim,
        obs_horizon,
        pred_horizon,
        action_horizon,
        training_data_stats,
        num_diffusion_iters,
        seed=4077,
    ):
        self.env = env
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.obs_horizon = obs_horizon
        self.pred_horizon = pred_horizon
        self.action_horizon = action_horizon
        self.training_data_stats = training_data_stats
        self.num_diffusion_iters = num_diffusion_iters

        self.seed = seed
        self.set_seed(self.seed)

        self.diffusion_network = ConditionalUnet1D(
            input_dim=action_dim, global_cond_dim=obs_dim * obs_horizon
        ).to(device)
        self.noise_scheduler = DDPMScheduler(
            num_train_timesteps=num_diffusion_iters,
            # the choice of beta schedule has big impact on performance
            # TRI found that squaredcos_cap_v2 works the best
            beta_schedule="squaredcos_cap_v2",
            # clip output to [-1,1] to improve stability
            clip_sample=True,
            # our network predicts noise (instead of denoised action)
            prediction_type="epsilon",
        )

        # Exponential Moving Average
        # accelerates training and improves stability
        # holds a copy of the model weights
        self.ema_model = EMAModel(
            parameters=self.diffusion_network.parameters(), power=0.75
        )

        # Standard ADAM optimizer
        # Note that EMA parametesr are not optimized
        self.optimizer = torch.optim.AdamW(
            params=self.diffusion_network.parameters(), lr=1e-4, weight_decay=1e-6
        )

    def train(self, num_epochs, dataloader):
        # Cosine LR schedule with linear warmup
        self.lr_scheduler = get_scheduler(
            name="cosine",
            optimizer=self.optimizer,
            num_warmup_steps=500,
            num_training_steps=len(dataloader) * num_epochs,
        )

        with tqdm(range(num_epochs), desc="Epoch") as tglobal:
            # epoch loop
            for epoch_idx in tglobal:
                epoch_loss = list()
                # batch loop
                with tqdm(dataloader, desc="Batch", leave=False) as tepoch:
                    for nbatch in tepoch:
                        # data normalized in dataset
                        # device transfer
                        norm_obs = nbatch["obs"].to(device)
                        norm_action = nbatch["action"].to(device)
                        B = norm_obs.shape[0]

                        # observation as FiLM conditioning
                        # (B, obs_horizon, obs_dim)
                        obs_cond = norm_obs[:, : self.obs_horizon, :]
                        # (B, obs_horizon * obs_dim)
                        obs_cond = obs_cond.flatten(start_dim=1)

                        # sample noise to add to actions
                        noise = torch.randn(norm_action.shape, device=device)

                        # sample a diffusion iteration for each data point
                        timesteps = torch.randint(
                            0,
                            self.noise_scheduler.config.num_train_timesteps,
                            (B,),
                            device=device,
                        ).long()

                        # add noise to the clean images according to the noise magnitude at each diffusion iteration
                        # (this is the forward diffusion process)
                        noisy_actions = self.noise_scheduler.add_noise(
                            norm_action, noise, timesteps
                        )

                        # predict the noise residual
                        noise_pred = self.diffusion_network(
                            noisy_actions, timesteps, global_cond=obs_cond
                        )

                        # L2 loss
                        loss = nn.functional.mse_loss(noise_pred, noise)

                        # optimize
                        loss.backward()
                        self.optimizer.step()
                        self.optimizer.zero_grad()
                        # step lr scheduler every batch
                        # this is different from standard pytorch behavior
                        self.lr_scheduler.step()

                        # update Exponential Moving Average of the model weights
                        self.ema_model.step(self.diffusion_network.parameters())

                        # logging
                        loss_cpu = loss.item()
                        epoch_loss.append(loss_cpu)
                        tepoch.set_postfix(loss=loss_cpu)
                    if epoch_idx % 5 == 0:
                        # save checkpoint every 5 epochs
                        self.save("./checkpoints")
                tglobal.set_postfix(loss=np.mean(epoch_loss))

    def inference(self, max_steps, render=False):
        env = self.env
        env.seed(self.seed)
        output_dir = hydra.core.hydra_config.HydraConfig.get().runtime.output_dir

        # get first observation
        obs, _ = env.reset()

        # keep a queue of last obs_horizon steps of observations
        obs_deque = collections.deque([obs] * self.obs_horizon, maxlen=self.obs_horizon)

        if render:
            imgs = [env.render(mode="rgb_array")]
        rewards = list()
        done = False
        step_idx = 0

        all_actions = []
        all_obs = [obs_deque[0]]

        with tqdm(total=max_steps, desc="Evaluation") as pbar:
            while not done:
                B = 1
                # stack the last obs_horizon number of observations
                obs_seq = np.stack(obs_deque)
                # normalize observation
                norm_obs = normalize_data(
                    obs_seq, stats=self.training_data_stats["obs"]
                )
                # device transfer
                norm_obs = torch.from_numpy(norm_obs).to(device, dtype=torch.float32)

                # infer action
                with torch.no_grad():
                    # reshape observation to (B,obs_horizon*obs_dim)
                    obs_cond = norm_obs.unsqueeze(0).flatten(start_dim=1)

                    # initialize action from Guassian noise
                    noisy_action = torch.randn(
                        (B, self.pred_horizon, self.action_dim), device=device
                    )
                    norm_action = noisy_action

                    # init scheduler
                    self.noise_scheduler.set_timesteps(self.num_diffusion_iters)

                    for k in self.noise_scheduler.timesteps:
                        # predict noise
                        noise_pred = self.diffusion_network(
                            sample=norm_action, timestep=k, global_cond=obs_cond
                        )

                        # inverse diffusion step (remove noise)
                        norm_action = self.noise_scheduler.step(
                            model_output=noise_pred, timestep=k, sample=norm_action
                        ).prev_sample

                # unnormalize action
                norm_action = norm_action.detach().to("cpu").numpy()
                # (B, pred_horizon, action_dim)
                norm_action = norm_action[0]
                action_pred = unnormalize_data(
                    norm_action, stats=self.training_data_stats["action"]
                )

                # only take action_horizon number of actions
                start = self.obs_horizon - 1
                end = start + self.action_horizon
                action = action_pred[start:end, :]
                # (action_horizon, action_dim)

                # execute action_horizon number of steps
                # without replanning
                for i in range(len(action)):
                    # stepping env
                    obs, reward, done, _, info = env.step(action[i])
                    all_actions.append(action[i])
                    all_obs.append(obs)

                    # save observations
                    obs_deque.append(obs)
                    # and reward
                    rewards.append(reward)

                    if render:
                        imgs.append(env.render(mode="rgb_array"))

                    # update progress bar
                    step_idx += 1
                    pbar.update(1)
                    pbar.set_postfix(reward=reward)
                    if step_idx > max_steps:
                        done = True
                    if done:
                        break

        # print out the maximum target coverage
        print("Score: ", max(rewards))

        np.save(f"{output_dir}/all_actions.npy", all_actions)
        np.save(f"{output_dir}/all_obs.npy", all_obs)

        if render:
            vwrite(f"{output_dir}/vis.mp4", imgs)
            Video(f"{output_dir}/vis.mp4", embed=True, width=256, height=256)

    def save(self, output_path):
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        torch.save(
            self.diffusion_network.state_dict(),
            os.path.join(output_path, "diffusion_policy.pt"),
        )

    def load(self, input_path):
        if not os.path.isfile(input_path):
            raise FileNotFoundError(f"File {input_path} not found.")

        state_dict = torch.load(input_path, map_location="cuda")
        self.diffusion_network.load_state_dict(state_dict)
        print("Pretrained weights loaded.")

    def set_seed(self, seed):
        np.random.seed(seed)
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.backends.cudnn.deterministic = True
            torch.backends.cudnn.benchmark = False
