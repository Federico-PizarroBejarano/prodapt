import collections
import numpy as np
import torch
import torch.nn as nn
from diffusers.training_utils import EMAModel
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from diffusers.optimization import get_scheduler
from tqdm.auto import tqdm
import os
from skvideo.io import vwrite

from prodapt.envs.push_t import PushTEnv
from prodapt.dataset.push_t_data import normalize_data, unnormalize_data
from prodapt.diffusion.conditional_unet_1d import ConditionalUnet1D
from prodapt.dataset.push_t_data import PushTStateDataset

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def create_push_t_dataloader(dataset_path, pred_horizon, obs_horizon, action_horizon):
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
    return dataloader, stats


class DiffusionPolicy:
    def __init__(
        self,
        obs_dim,
        action_dim,
        obs_horizon,
        training_data_stats,
        num_diffusion_iters=100,
    ):
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.obs_horizon = obs_horizon
        self.training_data_stats = training_data_stats
        self.diffusion_network = ConditionalUnet1D(
            input_dim=action_dim, global_cond_dim=obs_dim * obs_horizon
        ).to(device)
        self.num_diffusion_iters = num_diffusion_iters
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

    def train(self, num_epochs):
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
                        nobs = nbatch["obs"].to(device)
                        naction = nbatch["action"].to(device)
                        B = nobs.shape[0]

                        # observation as FiLM conditioning
                        # (B, obs_horizon, obs_dim)
                        obs_cond = nobs[:, :obs_horizon, :]
                        # (B, obs_horizon * obs_dim)
                        obs_cond = obs_cond.flatten(start_dim=1)

                        # sample noise to add to actions
                        noise = torch.randn(naction.shape, device=device)

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
                            naction, noise, timesteps
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
                        if epoch_idx % 5 == 0:
                            # save checkpoint every 5 epochs
                            self.save("./checkpoints")

                        # update Exponential Moving Average of the model weights
                        self.ema_model.step(self.diffusion_network.parameters())

                        # logging
                        loss_cpu = loss.item()
                        epoch_loss.append(loss_cpu)
                        tepoch.set_postfix(loss=loss_cpu)
                tglobal.set_postfix(loss=np.mean(epoch_loss))

    def inference(self):
        ema_diffusion_network = self.diffusion_network
        self.ema_model.copy_to(ema_diffusion_network.parameters())

        # limit enviornment interaction to 200 steps before termination
        max_steps = 200
        env = PushTEnv()
        # use a seed >200 to avoid initial states seen in the training dataset
        env.seed(100000)

        # get first observation
        obs, info = env.reset()

        # keep a queue of last 2 steps of observations
        obs_deque = collections.deque([obs] * obs_horizon, maxlen=obs_horizon)
        # save visualization and rewards
        imgs = [env.render(mode="rgb_array")]
        rewards = list()
        done = False
        step_idx = 0

        with tqdm(total=max_steps, desc="Eval PushTStateEnv") as pbar:
            while not done:
                B = 1
                # stack the last obs_horizon (2) number of observations
                obs_seq = np.stack(obs_deque)
                # normalize observation
                nobs = normalize_data(obs_seq, stats=self.training_data_stats["obs"])
                # device transfer
                nobs = torch.from_numpy(nobs).to(device, dtype=torch.float32)

                # infer action
                with torch.no_grad():
                    # reshape observation to (B,obs_horizon*obs_dim)
                    obs_cond = nobs.unsqueeze(0).flatten(start_dim=1)

                    # initialize action from Guassian noise
                    noisy_action = torch.randn(
                        (B, pred_horizon, action_dim), device=device
                    )
                    naction = noisy_action

                    # init scheduler
                    self.noise_scheduler.set_timesteps(self.num_diffusion_iters)

                    for k in self.noise_scheduler.timesteps:
                        # predict noise
                        noise_pred = self.diffusion_network(
                            sample=naction, timestep=k, global_cond=obs_cond
                        )

                        # inverse diffusion step (remove noise)
                        naction = self.noise_scheduler.step(
                            model_output=noise_pred, timestep=k, sample=naction
                        ).prev_sample

                # unnormalize action
                naction = naction.detach().to("cpu").numpy()
                # (B, pred_horizon, action_dim)
                naction = naction[0]
                action_pred = unnormalize_data(
                    naction, stats=self.training_data_stats["action"]
                )

                # only take action_horizon number of actions
                start = obs_horizon - 1
                end = start + action_horizon
                action = action_pred[start:end, :]
                # (action_horizon, action_dim)

                # execute action_horizon number of steps
                # without replanning
                for i in range(len(action)):
                    # stepping env
                    obs, reward, done, _, info = env.step(action[i])
                    # save observations
                    obs_deque.append(obs)
                    # and reward/vis
                    rewards.append(reward)
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

        # visualize
        from IPython.display import Video

        vwrite("vis.mp4", imgs)
        Video("vis.mp4", embed=True, width=256, height=256)

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


if __name__ == "__main__":
    obs_dim = 5
    action_dim = 2
    obs_horizon = 2

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
        obs_dim, action_dim, obs_horizon, stats, num_diffusion_iters=100
    )
    diffusion_policy.load("./checkpoints/pusht_state_100ep.ckpt")
    diffusion_policy.inference()
