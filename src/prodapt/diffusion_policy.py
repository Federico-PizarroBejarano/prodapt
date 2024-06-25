import collections
import os
import pickle
import select
import socket

import hydra
import numpy as np
import torch
import torch.nn as nn
from diffusers.optimization import get_scheduler
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from diffusers.training_utils import EMAModel
from IPython.display import Video
from skvideo.io import vwrite
from tqdm.auto import tqdm

from prodapt.dataset.dataset_utils import normalize_data, unnormalize_data
from prodapt.diffusion.conditional_unet_1d import ConditionalUnet1D
from prodapt.diffusion.transformer_for_diffusion import TransformerForDiffusion

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
        use_transformer=False,
        network_args=None,
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

        self.use_transformer = use_transformer

        if not self.use_transformer:
            self.diffusion_network = ConditionalUnet1D(
                input_dim=action_dim,
                global_cond_dim=obs_dim * obs_horizon,
                **network_args,
            ).to(device)
        else:
            self.diffusion_network = TransformerForDiffusion(
                input_dim=action_dim,
                output_dim=action_dim,
                horizon=pred_horizon,
                n_obs_steps=obs_horizon,
                cond_dim=obs_dim if network_args["obs_as_cond"] else 0,
                **network_args,
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

    def train(self, num_epochs, dataloader, checkpoint_path):
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
                        if not self.use_transformer:
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
                        self.save(checkpoint_path)
                tglobal.set_postfix(loss=np.mean(epoch_loss))

    def evaluate(self, num_inferences, max_steps, render=False, warmstart=False):
        total_results = {"rewards": [], "return": [], "done": []}
        output_dir = hydra.core.hydra_config.HydraConfig.get().runtime.output_dir

        # Socket to send environment reset requests
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setblocking(0)
        sock.connect_ex((socket.gethostname(), 6000))

        select.select([], [sock], [])

        for inf_id in range(num_inferences):
            sock.send(bytes("Reset environment", "UTF-8"))
            results = self.inference(max_steps, output_dir, inf_id, render, warmstart)
            for key in total_results.keys():
                total_results[key].append(results[key])

        final_return = np.mean(total_results["return"])
        final_done = np.mean(total_results["done"])

        print(f"Mean Return: {final_return}")
        print(f"Mean Done: {final_done}")

        with open(f"{output_dir}/total_results.pkl", "wb") as handle:
            pickle.dump(total_results, handle, protocol=pickle.HIGHEST_PROTOCOL)

    def inference(
        self, max_steps, output_dir, inference_id, render=False, warmstart=False
    ):
        env = self.env
        env.seed(self.seed)

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

        prev_action_traj = None
        colliding = False

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
                    obs_cond = norm_obs.unsqueeze(0)

                    if not self.use_transformer:
                        obs_cond = obs_cond.flatten(start_dim=1)

                    # initialize action from Guassian noise
                    noise = torch.randn(
                        (B, self.pred_horizon, self.action_dim), device=device
                    )
                    if not warmstart or prev_action_traj is None or colliding:
                        noisy_action = noise
                    else:
                        noisy_action = torch.concatenate(
                            (
                                prev_action_traj[:, self.action_horizon :, :],
                                noise[:, : self.action_horizon, :],
                            ),
                            axis=1,
                        )
                        noisy_action[self.action_horizon :] += (
                            noise[self.action_horizon :] * 0.5
                        )
                    norm_action = noisy_action

                    # init scheduler
                    if not warmstart or prev_action_traj is None or colliding:
                        self.noise_scheduler.set_timesteps(self.num_diffusion_iters)
                    else:
                        self.noise_scheduler.set_timesteps(
                            self.num_diffusion_iters // 2
                        )

                    for k in self.noise_scheduler.timesteps:
                        # predict noise
                        noise_pred = self.diffusion_network(
                            sample=norm_action, timestep=k, global_cond=obs_cond
                        )

                        # inverse diffusion step (remove noise)
                        norm_action = self.noise_scheduler.step(
                            model_output=noise_pred, timestep=k, sample=norm_action
                        ).prev_sample

                    prev_action_traj = norm_action

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
                action = action_pred[start:end, :]  # (action_horizon, action_dim)

                # execute action_horizon number of steps
                # without replanning
                for i in range(len(action)):
                    # stepping env
                    next_action = self.post_process_action(action, all_actions, i)
                    obs, reward, done, _, info = env.step(next_action)
                    all_actions.append(next_action)
                    all_obs.append(obs)

                    if "kp_added" in info:
                        if info["kp_added"] and not colliding:
                            colliding = True
                        else:
                            colliding = False

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
                    if colliding:
                        break

        # print out the maximum target coverage
        print("Score: ", sum(rewards))

        results = {"rewards": rewards, "return": sum(rewards), "done": done}

        os.makedirs(f"{output_dir}/{inference_id}")

        np.save(f"{output_dir}/{inference_id}/all_actions.npy", all_actions)
        np.save(f"{output_dir}/{inference_id}/all_obs.npy", all_obs)

        if render:
            vwrite(f"{output_dir}/{inference_id}/vis.mp4", imgs)
            Video(
                f"{output_dir}/{inference_id}/vis.mp4",
                embed=True,
                width=256,
                height=256,
            )

        return results

    def save(self, output_path):
        folder_name = output_path[: -output_path[::-1].index("/")]
        if not os.path.exists(folder_name):
            os.makedirs(folder_name)
        torch.save(self.diffusion_network.state_dict(), output_path)

    def load(self, input_path):
        if not os.path.isfile(input_path):
            raise FileNotFoundError(f"File {input_path} not found.")

        state_dict = torch.load(input_path, map_location=device)
        self.diffusion_network.load_state_dict(state_dict)
        print("Pretrained weights loaded.")

    def set_seed(self, seed):
        np.random.seed(seed)
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.backends.cudnn.deterministic = True
            torch.backends.cudnn.benchmark = False

    def post_process_action(self, action, all_actions, iter):
        if len(all_actions) == 0:
            next_action = action[iter]
        else:
            next_action = 0.3 * all_actions[-1] + 0.7 * action[iter]

        return next_action
