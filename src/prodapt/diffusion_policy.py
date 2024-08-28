import collections
import os
import pickle
import zmq
import time

import hydra
import numpy as np
import torch
import torch.nn as nn
from diffusers.optimization import get_scheduler
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from diffusers.training_utils import EMAModel
from IPython.display import Video
import matplotlib.pyplot as plt
from skvideo.io import vwrite
from tqdm.auto import tqdm

from prodapt.dataset.dataset_utils import normalize_data, unnormalize_data
from prodapt.diffusion.conditional_unet_1d import ConditionalUnet1D
from prodapt.diffusion.transformer_for_diffusion import TransformerForDiffusion

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


class DiffusionPolicy:
    def __init__(
        self,
        obs_dim,
        real_obs_dim,
        action_dim,
        obs_horizon,
        pred_horizon,
        action_horizon,
        training_data_stats,
        num_diffusion_iters,
        seed=4077,
        use_transformer=False,
        network_args=None,
        num_keypoints=0,
    ):
        self.obs_dim = obs_dim
        self.real_obs_dim = real_obs_dim
        self.action_dim = action_dim
        self.obs_horizon = obs_horizon
        self.pred_horizon = pred_horizon
        self.action_horizon = action_horizon
        self.training_data_stats = training_data_stats
        self.num_diffusion_iters = num_diffusion_iters
        self.num_keypoints = num_keypoints

        self.seed = seed
        self.set_seed(self.seed)

        self.use_transformer = use_transformer

        if not self.use_transformer:
            self.diffusion_network = ConditionalUnet1D(
                input_dim=action_dim,
                global_cond_dim=real_obs_dim * obs_horizon
                + real_obs_dim * num_keypoints,
                **network_args,
            ).to(device)
        else:
            self.diffusion_network = TransformerForDiffusion(
                input_dim=action_dim,
                output_dim=action_dim,
                horizon=pred_horizon,
                n_obs_steps=obs_horizon + num_keypoints,
                cond_dim=real_obs_dim,
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

    def train(self, num_epochs, dataloader, model_name):
        # Cosine LR schedule with linear warmup
        self.lr_scheduler = get_scheduler(
            name="cosine",
            optimizer=self.optimizer,
            num_warmup_steps=500,
            num_training_steps=len(dataloader) * num_epochs,
        )

        losses = []

        with tqdm(range(num_epochs), desc="Epoch") as tglobal:
            # epoch loop
            for epoch_idx in tglobal:
                epoch_loss = list()
                # batch loop
                with tqdm(
                    dataloader, desc="Batch", leave=False, disable=True
                ) as tepoch:
                    for nbatch in tepoch:
                        # data normalized in dataset
                        # device transfer (B, obs_horizon * obs_dim)
                        norm_obs = nbatch["obs"].to(device)
                        norm_action = nbatch["action"].to(device)
                        B = norm_obs.shape[0]

                        # Observation as FiLM conditioning
                        # (B, obs_horizon * real_obs_dim + num_keypoints * real_obs_dim)
                        obs_cond = self.transform_obs_cond(norm_obs)

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
                            noisy_actions,
                            timesteps,
                            global_cond=obs_cond,
                            obs_horizon=self.obs_horizon,
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
                        self.save(model_name)
                tglobal.set_postfix(loss=np.mean(epoch_loss))
                losses.append(np.mean(epoch_loss))

        plt.plot(losses[2:])
        plt.xlabel("Epochs")
        plt.ylabel("Mean Loss")
        plt.savefig(f"./checkpoints/{model_name}/losses.png")

    def evaluate(
        self, env, num_inferences, max_steps, model_name, render=False, warmstart=False
    ):
        env = env
        total_results = {"iters": [], "done": [], "time": [], "diff_times": []}
        output_dir = hydra.core.hydra_config.HydraConfig.get().runtime.output_dir

        close = False
        if env.name == "ur10" and env.interface == "isaacsim":
            # Socket to send environment reset requests
            context = zmq.Context()
            self.sock = context.socket(zmq.REQ)
            self.sock.connect("tcp://localhost:5555")

        for inf_id in range(num_inferences):
            print(f"Starting Inference #{inf_id+1}")
            if env.name == "ur10" and env.interface == "isaacsim":
                self.sock.send(bytes("reset", "UTF-8"))
                while True:
                    try:
                        msg = self.sock.recv().decode()
                        if msg == "reset":
                            break
                        if msg == "close":
                            close = True
                            break
                    except:
                        pass
            if close:
                break
            results = self.inference(
                env, max_steps, output_dir, inf_id, render, warmstart
            )
            for key in total_results.keys():
                total_results[key].append(results[key])

        if env.name == "ur10" and env.interface == "isaacsim":
            self.sock.close()

        print(total_results["iters"])
        print(total_results["done"])

        with open(f"./results/{model_name}.pkl", "wb") as handle:
            pickle.dump(total_results, handle, protocol=pickle.HIGHEST_PROTOCOL)

    def inference(
        self, env, max_steps, output_dir, inference_id, render=False, warmstart=False
    ):
        env.seed(self.seed)

        # get first observation
        obs, _ = env.reset()

        # keep a queue of last obs_horizon steps of observations
        obs_deque = collections.deque([obs] * self.obs_horizon, maxlen=self.obs_horizon)

        if render:
            imgs = [env.render(mode="rgb_array")]
        done = False
        step_idx = 0

        all_actions = []
        all_obs = [obs_deque[0]]

        prev_action_traj = None
        warmstart_div = 3

        start_time = time.time()
        prev_time = start_time
        all_diff_times = []

        with tqdm(total=max_steps, desc="Evaluation") as pbar:
            while not (done and step_idx > 50) and step_idx < max_steps:
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
                    obs_cond = self.transform_obs_cond(obs_cond)

                    if not self.use_transformer:
                        obs_cond = obs_cond.flatten(start_dim=1)

                    # initialize action from Guassian noise
                    noise = torch.randn(
                        (B, self.pred_horizon, self.action_dim), device=device
                    )
                    if not warmstart or prev_action_traj is None:
                        noisy_action = noise
                    else:
                        timestep = self.noise_scheduler.timesteps[
                            [-self.num_diffusion_iters // warmstart_div]
                        ]
                        norm_action = torch.concatenate(
                            (
                                prev_action_traj[:, self.action_horizon :, :],
                                torch.tile(
                                    prev_action_traj[:, -1, :],
                                    (1, self.action_horizon, 1),
                                ),
                            ),
                            axis=1,
                        )

                        noisy_action = self.noise_scheduler.add_noise(
                            norm_action, noise, timestep
                        )
                    norm_action = noisy_action

                    if not warmstart or prev_action_traj is None:
                        timesteps = self.noise_scheduler.timesteps
                    else:
                        timesteps = self.noise_scheduler.timesteps[
                            -self.num_diffusion_iters // warmstart_div :
                        ]

                    diff_start_time = time.time()
                    for k in timesteps:
                        # predict noise
                        noise_pred = self.diffusion_network(
                            sample=norm_action,
                            timestep=k,
                            global_cond=obs_cond,
                            obs_horizon=self.obs_horizon,
                        )

                        # inverse diffusion step (remove noise)
                        norm_action = self.noise_scheduler.step(
                            model_output=noise_pred, timestep=k, sample=norm_action
                        ).prev_sample
                    diff_time = time.time() - diff_start_time
                    all_diff_times.append(diff_time)

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
                assert action.shape[0] == self.action_horizon

                # execute action_horizon number of steps without replanning
                for i in range(len(action)):
                    # Set rate at 10Hz
                    time.sleep(max(0, 0.1 - (time.time() - prev_time)))
                    prev_time = time.time()

                    # stepping env
                    next_action = self.post_process_action(action, all_actions, i)
                    obs, _, done, _, _ = env.step(next_action)
                    all_actions.append(next_action)
                    all_obs.append(obs)

                    # save observations
                    obs_deque.append(obs)

                    if render:
                        imgs.append(env.render(mode="rgb_array"))

                    # update progress bar
                    step_idx += 1
                    pbar.update(1)
                    if step_idx >= max_steps:
                        break
                    if done and step_idx > 50:
                        break

        print("Total Iters: ", step_idx)

        results = {
            "iters": step_idx,
            "done": done,
            "time": time.time() - start_time,
            "diff_times": all_diff_times,
        }

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

    def save(self, model_name):
        folder_name = f"./checkpoints/{model_name}"
        if not os.path.exists(folder_name):
            os.makedirs(folder_name)
        torch.save(
            self.diffusion_network.state_dict(), f"{folder_name}/{model_name}.pt"
        )

    def load(self, model_name, input_path):
        if input_path is None:
            input_path = f"./checkpoints/{model_name}/{model_name}.pt"
        if not os.path.isfile(input_path):
            raise FileNotFoundError(f"File {input_path} not found.")

        state_dict = torch.load(input_path, map_location=device)
        self.diffusion_network.load_state_dict(state_dict)

    def set_seed(self, seed):
        np.random.seed(seed)
        torch.manual_seed(seed)
        # if torch.cuda.is_available():
        #     torch.backends.cudnn.deterministic = True
        #     torch.backends.cudnn.benchmark = False

    def post_process_action(self, action, all_actions, iter):
        if len(all_actions) == 0:
            next_action = action[iter]
        else:
            next_action = 0.3 * all_actions[-1] + 0.7 * action[iter]

        return next_action

    def transform_obs_cond(self, obs_cond):
        if self.num_keypoints > 0:
            real_obs = obs_cond[:, :, : self.real_obs_dim]
            keypoint_obs = obs_cond[:, -1, self.real_obs_dim :].reshape(
                -1, self.num_keypoints, self.real_obs_dim
            )

            trans_obs_cond = torch.concat((real_obs, keypoint_obs), axis=1)

            return trans_obs_cond
        else:
            return obs_cond
