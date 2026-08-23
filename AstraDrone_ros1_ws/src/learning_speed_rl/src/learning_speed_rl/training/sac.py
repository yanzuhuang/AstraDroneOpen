"""Minimal PyTorch SAC learner adapted from the reviewed CleanRL structure."""

from dataclasses import asdict, dataclass
import math
import threading
import time

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as functional
import torch.optim as optim

from .sac_replay import ACTION_DIM, OBSERVATION_DIM


@dataclass(frozen=True)
class SacConfig:
    observation_dim: int = OBSERVATION_DIM
    action_dim: int = ACTION_DIM
    hidden_dim: int = 256
    gamma: float = 0.99
    tau: float = 0.005
    batch_size: int = 64
    # Calibrated for the frozen, unnormalised 3267-D Observation C input.  The
    # former 3e-4 value moved the pre-tanh mean by more than 12 in one update.
    policy_learning_rate: float = 1.0e-5
    critic_learning_rate: float = 1.0e-3
    alpha_learning_rate: float = 1.0e-3
    policy_frequency: int = 2
    critic_warmup_updates: int = 100
    target_network_frequency: int = 1
    # AstraDroneOpen exploration-stability envelope.  The former CleanRL-style
    # [-5, 2] range let the frozen, unnormalised Observation C drive the
    # log-std head between both saturation limits during the 10k pilot.
    log_std_min: float = -3.0
    log_std_max: float = -1.0
    automatic_entropy_tuning: bool = True
    initial_alpha: float = 0.2
    target_entropy: float = -1.0
    device: str = "cpu"
    torch_num_threads: int = 1
    seed: int = 1
    learner_side_normalization: bool = False

    def validate(self):
        if self.observation_dim != OBSERVATION_DIM or self.action_dim != 1:
            raise ValueError("SAC must preserve the 3267-D/1-D contract")
        if self.hidden_dim <= 0 or self.batch_size <= 0:
            raise ValueError("SAC network/batch sizes must be positive")
        if not 0.0 < self.gamma <= 1.0 or not 0.0 < self.tau <= 1.0:
            raise ValueError("SAC gamma/tau are invalid")
        if min(
            self.policy_learning_rate,
            self.critic_learning_rate,
            self.alpha_learning_rate,
            self.initial_alpha,
        ) <= 0.0:
            raise ValueError("SAC learning rates/alpha must be positive")
        if (
            self.policy_frequency <= 0
            or self.critic_warmup_updates < 0
            or self.target_network_frequency <= 0
        ):
            raise ValueError("SAC update frequencies must be positive")
        if self.log_std_max <= self.log_std_min:
            raise ValueError("SAC log-std bounds are invalid")
        if self.torch_num_threads <= 0:
            raise ValueError("torch_num_threads must be positive")
        if self.learner_side_normalization:
            raise ValueError(
                "learner-side normalization is intentionally disabled"
            )


class SoftQNetwork(nn.Module):
    def __init__(self, observation_dim, action_dim, hidden_dim):
        super().__init__()
        self.fc1 = nn.Linear(observation_dim + action_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, 1)

    def forward(self, observation, action):
        value = torch.cat((observation, action), dim=1)
        value = functional.relu(self.fc1(value))
        value = functional.relu(self.fc2(value))
        return self.fc3(value)


class GaussianActor(nn.Module):
    def __init__(self, config):
        super().__init__()
        self.fc1 = nn.Linear(config.observation_dim, config.hidden_dim)
        self.fc2 = nn.Linear(config.hidden_dim, config.hidden_dim)
        self.fc_mean = nn.Linear(config.hidden_dim, config.action_dim)
        self.fc_log_std = nn.Linear(config.hidden_dim, config.action_dim)
        self.log_std_min = float(config.log_std_min)
        self.log_std_max = float(config.log_std_max)

    def forward(self, observation):
        value = functional.relu(self.fc1(observation))
        value = functional.relu(self.fc2(value))
        mean = self.fc_mean(value)
        log_std = torch.tanh(self.fc_log_std(value))
        log_std = self.log_std_min + 0.5 * (
            self.log_std_max - self.log_std_min
        ) * (log_std + 1.0)
        return mean, log_std

    def get_action(self, observation):
        mean, log_std = self(observation)
        distribution = torch.distributions.Normal(mean, log_std.exp())
        sample = distribution.rsample()
        action = torch.tanh(sample)
        log_probability = distribution.log_prob(sample)
        log_probability -= torch.log(1.0 - action.pow(2) + 1.0e-6)
        log_probability = log_probability.sum(dim=1, keepdim=True)
        deterministic = torch.tanh(mean)
        return action, log_probability, deterministic


class SacAgent:
    """Twin-Q SAC with target critics and automatic entropy tuning."""

    def __init__(self, config):
        if not isinstance(config, SacConfig):
            raise TypeError("config must be SacConfig")
        config.validate()
        self.config = config
        torch.set_num_threads(config.torch_num_threads)
        np.random.seed(config.seed)
        torch.manual_seed(config.seed)
        self.device = torch.device(config.device)
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("configured CUDA device is unavailable")
        self.lock = threading.RLock()
        self.actor = GaussianActor(config).to(self.device)
        self.critic1 = SoftQNetwork(
            config.observation_dim, config.action_dim, config.hidden_dim
        ).to(self.device)
        self.critic2 = SoftQNetwork(
            config.observation_dim, config.action_dim, config.hidden_dim
        ).to(self.device)
        self.target_critic1 = SoftQNetwork(
            config.observation_dim, config.action_dim, config.hidden_dim
        ).to(self.device)
        self.target_critic2 = SoftQNetwork(
            config.observation_dim, config.action_dim, config.hidden_dim
        ).to(self.device)
        self.target_critic1.load_state_dict(self.critic1.state_dict())
        self.target_critic2.load_state_dict(self.critic2.state_dict())
        self.critic_optimizer = optim.Adam(
            list(self.critic1.parameters()) + list(self.critic2.parameters()),
            lr=config.critic_learning_rate,
        )
        self.actor_optimizer = optim.Adam(
            self.actor.parameters(), lr=config.policy_learning_rate
        )
        self.log_alpha = torch.tensor(
            [math.log(config.initial_alpha)],
            dtype=torch.float32,
            requires_grad=config.automatic_entropy_tuning,
            device=self.device,
        )
        self.alpha_optimizer = (
            optim.Adam([self.log_alpha], lr=config.alpha_learning_rate)
            if config.automatic_entropy_tuning
            else None
        )
        self.update_step = 0
        self.global_environment_step = 0
        self._initial_actor = {
            key: value.detach().cpu().clone()
            for key, value in self.actor.state_dict().items()
        }
        self._initial_critic1 = {
            key: value.detach().cpu().clone()
            for key, value in self.critic1.state_dict().items()
        }

    @property
    def alpha(self):
        return float(self.log_alpha.detach().exp().cpu().item())

    @staticmethod
    def _all_finite(parameters):
        return all(
            bool(torch.all(torch.isfinite(parameter)).item())
            for parameter in parameters
        )

    @staticmethod
    def _gradient_statistics(parameters):
        gradients = [
            parameter.grad.detach()
            for parameter in parameters
            if parameter.grad is not None
        ]
        if not gradients:
            return True, 0.0
        finite = all(bool(torch.all(torch.isfinite(value)).item()) for value in gradients)
        maximum = max(float(value.abs().max().cpu().item()) for value in gradients)
        return finite, maximum

    def sample_action(self, observation, deterministic=False):
        value = np.asarray(observation, dtype=np.float32)
        if value.shape != (self.config.observation_dim,) or not np.all(
            np.isfinite(value)
        ):
            raise ValueError("SAC observation must be finite and 3267-D")
        with self.lock, torch.no_grad():
            tensor = torch.as_tensor(value, device=self.device).unsqueeze(0)
            sampled, _, mean = self.actor.get_action(tensor)
            action = mean if deterministic else sampled
            result = float(action.squeeze().cpu().item())
        if not math.isfinite(result) or result < -1.0 or result > 1.0:
            raise RuntimeError("SAC Actor produced an invalid action")
        return result

    def update(self, batch):
        with self.lock:
            observations = torch.as_tensor(
                batch["observations"], dtype=torch.float32, device=self.device
            )
            next_observations = torch.as_tensor(
                batch["next_observations"], dtype=torch.float32,
                device=self.device,
            )
            actions = torch.as_tensor(
                batch["actions"], dtype=torch.float32, device=self.device
            )
            rewards = torch.as_tensor(
                batch["rewards"], dtype=torch.float32, device=self.device
            ).view(-1)
            terminated = torch.as_tensor(
                batch["terminated"], dtype=torch.float32, device=self.device
            ).view(-1)
            truncated = torch.as_tensor(
                batch["truncated"], dtype=torch.float32, device=self.device
            ).view(-1)

            with torch.no_grad():
                next_actions, next_log_probability, _ = self.actor.get_action(
                    next_observations
                )
                target_q1 = self.target_critic1(
                    next_observations, next_actions
                )
                target_q2 = self.target_critic2(
                    next_observations, next_actions
                )
                minimum_target_q = torch.min(target_q1, target_q2)
                minimum_target_q -= self.alpha * next_log_probability
                # Time-limit truncation bootstraps; true termination does not.
                target = rewards + (1.0 - terminated) * self.config.gamma * (
                    minimum_target_q.view(-1)
                )

            q1 = self.critic1(observations, actions).view(-1)
            q2 = self.critic2(observations, actions).view(-1)
            critic1_loss = functional.mse_loss(q1, target)
            critic2_loss = functional.mse_loss(q2, target)
            critic_loss = critic1_loss + critic2_loss
            self.critic_optimizer.zero_grad()
            critic_loss.backward()
            critic_gradient_finite, critic_gradient_max = (
                self._gradient_statistics(
                    list(self.critic1.parameters())
                    + list(self.critic2.parameters())
                )
            )
            if not critic_gradient_finite:
                raise FloatingPointError("critic gradient contains NaN/Inf")
            self.critic_optimizer.step()

            actor_loss_value = None
            alpha_loss_value = None
            entropy_value = None
            actor_gradient_max = None
            actor_action_statistics = {
                "actor_action_mean": None,
                "actor_action_std": None,
                "actor_action_min": None,
                "actor_action_max": None,
                "actor_deterministic_action_mean": None,
                "actor_deterministic_action_std": None,
                "actor_deterministic_action_min": None,
                "actor_deterministic_action_max": None,
                "actor_mean_pre_tanh_mean": None,
                "actor_mean_pre_tanh_std": None,
                "actor_log_std_mean": None,
                "actor_log_std_std": None,
                "actor_log_std_min": None,
                "actor_log_std_max": None,
                "actor_sample_std_mean": None,
            }
            actor_update_enabled = (
                self.update_step >= self.config.critic_warmup_updates
            )
            actor_update_due = actor_update_enabled and (
                (self.update_step - self.config.critic_warmup_updates)
                % self.config.policy_frequency
                == 0
            )
            if actor_update_due:
                policy_action, log_probability, _ = self.actor.get_action(
                    observations
                )
                actor_mean_pre_tanh, actor_log_std = self.actor(observations)
                actor_deterministic_action = torch.tanh(actor_mean_pre_tanh)
                minimum_policy_q = torch.min(
                    self.critic1(observations, policy_action),
                    self.critic2(observations, policy_action),
                )
                actor_loss = (
                    self.alpha * log_probability - minimum_policy_q
                ).mean()
                self.actor_optimizer.zero_grad()
                actor_loss.backward()
                actor_gradient_finite, actor_gradient_max = (
                    self._gradient_statistics(self.actor.parameters())
                )
                if not actor_gradient_finite:
                    raise FloatingPointError("actor gradient contains NaN/Inf")
                self.actor_optimizer.step()
                actor_loss_value = float(actor_loss.detach().cpu().item())
                entropy_value = float((-log_probability).mean().detach().cpu().item())
                actor_action_statistics = {
                    "actor_action_mean": float(
                        policy_action.mean().detach().cpu().item()
                    ),
                    "actor_action_std": float(
                        policy_action.std(unbiased=False).detach().cpu().item()
                    ),
                    "actor_action_min": float(
                        policy_action.min().detach().cpu().item()
                    ),
                    "actor_action_max": float(
                        policy_action.max().detach().cpu().item()
                    ),
                    "actor_deterministic_action_mean": float(
                        actor_deterministic_action.mean().detach().cpu().item()
                    ),
                    "actor_deterministic_action_std": float(
                        actor_deterministic_action.std(
                            unbiased=False
                        ).detach().cpu().item()
                    ),
                    "actor_deterministic_action_min": float(
                        actor_deterministic_action.min().detach().cpu().item()
                    ),
                    "actor_deterministic_action_max": float(
                        actor_deterministic_action.max().detach().cpu().item()
                    ),
                    "actor_mean_pre_tanh_mean": float(
                        actor_mean_pre_tanh.mean().detach().cpu().item()
                    ),
                    "actor_mean_pre_tanh_std": float(
                        actor_mean_pre_tanh.std(
                            unbiased=False
                        ).detach().cpu().item()
                    ),
                    "actor_log_std_mean": float(
                        actor_log_std.mean().detach().cpu().item()
                    ),
                    "actor_log_std_std": float(
                        actor_log_std.std(unbiased=False).detach().cpu().item()
                    ),
                    "actor_log_std_min": float(
                        actor_log_std.min().detach().cpu().item()
                    ),
                    "actor_log_std_max": float(
                        actor_log_std.max().detach().cpu().item()
                    ),
                    "actor_sample_std_mean": float(
                        actor_log_std.exp().mean().detach().cpu().item()
                    ),
                }

                if self.config.automatic_entropy_tuning:
                    with torch.no_grad():
                        _, alpha_log_probability, _ = self.actor.get_action(
                            observations
                        )
                    alpha_loss = (
                        -self.log_alpha.exp()
                        * (
                            alpha_log_probability
                            + self.config.target_entropy
                        )
                    ).mean()
                    self.alpha_optimizer.zero_grad()
                    alpha_loss.backward()
                    if not bool(torch.all(torch.isfinite(self.log_alpha.grad)).item()):
                        raise FloatingPointError("alpha gradient contains NaN/Inf")
                    self.alpha_optimizer.step()
                    alpha_loss_value = float(alpha_loss.detach().cpu().item())

            if self.update_step % self.config.target_network_frequency == 0:
                with torch.no_grad():
                    for source, target_parameter in zip(
                        self.critic1.parameters(), self.target_critic1.parameters()
                    ):
                        target_parameter.mul_(1.0 - self.config.tau)
                        target_parameter.add_(self.config.tau * source)
                    for source, target_parameter in zip(
                        self.critic2.parameters(), self.target_critic2.parameters()
                    ):
                        target_parameter.mul_(1.0 - self.config.tau)
                        target_parameter.add_(self.config.tau * source)

            self.update_step += 1
            tensors = (critic1_loss, critic2_loss, target, q1, q2)
            if not all(bool(torch.all(torch.isfinite(value)).item()) for value in tensors):
                raise FloatingPointError("SAC update produced NaN/Inf")
            if not self._all_finite(
                list(self.actor.parameters())
                + list(self.critic1.parameters())
                + list(self.critic2.parameters())
                + list(self.target_critic1.parameters())
                + list(self.target_critic2.parameters())
                + [self.log_alpha]
            ):
                raise FloatingPointError("SAC parameters contain NaN/Inf")
            metrics = {
                "update_step": self.update_step,
                "actor_update_enabled": actor_update_enabled,
                "actor_update_applied": actor_update_due,
                "critic_warmup_updates": self.config.critic_warmup_updates,
                "critic1_loss": float(critic1_loss.detach().cpu().item()),
                "critic2_loss": float(critic2_loss.detach().cpu().item()),
                "actor_loss": actor_loss_value,
                "alpha_loss": alpha_loss_value,
                "alpha": self.alpha,
                "entropy": entropy_value,
                "q_mean": float(torch.cat((q1, q2)).mean().detach().cpu().item()),
                "q_min": float(torch.min(torch.cat((q1, q2))).detach().cpu().item()),
                "q_max": float(torch.max(torch.cat((q1, q2))).detach().cpu().item()),
                "q1_mean": float(q1.mean().detach().cpu().item()),
                "q1_min": float(q1.min().detach().cpu().item()),
                "q1_max": float(q1.max().detach().cpu().item()),
                "q2_mean": float(q2.mean().detach().cpu().item()),
                "q2_min": float(q2.min().detach().cpu().item()),
                "q2_max": float(q2.max().detach().cpu().item()),
                "target_q_mean": float(target.mean().detach().cpu().item()),
                "target_q_min": float(target.min().detach().cpu().item()),
                "target_q_max": float(target.max().detach().cpu().item()),
                "critic_gradient_max_abs": critic_gradient_max,
                "actor_gradient_max_abs": actor_gradient_max,
                "gradient_finite": True,
                "truncated_batch_count": int(truncated.sum().cpu().item()),
                "terminated_batch_count": int(terminated.sum().cpu().item()),
            }
            metrics.update(actor_action_statistics)
            return metrics

    @staticmethod
    def _parameter_delta(module, initial):
        total = 0.0
        for key, value in module.state_dict().items():
            difference = value.detach().cpu().double() - initial[key].double()
            total += float(torch.sum(difference * difference).item())
        return math.sqrt(total)

    def parameter_update_audit(self):
        with self.lock:
            return {
                "actor_l2_delta": self._parameter_delta(
                    self.actor, self._initial_actor
                ),
                "critic1_l2_delta": self._parameter_delta(
                    self.critic1, self._initial_critic1
                ),
            }

    def save_checkpoint(self, path, extra_config=None):
        with self.lock:
            payload = {
                "version": "astradrone_sac_checkpoint_v1.0",
                "config": asdict(self.config),
                "extra_config": dict(extra_config or {}),
                "actor": self.actor.state_dict(),
                "critic1": self.critic1.state_dict(),
                "critic2": self.critic2.state_dict(),
                "target_critic1": self.target_critic1.state_dict(),
                "target_critic2": self.target_critic2.state_dict(),
                "actor_optimizer": self.actor_optimizer.state_dict(),
                "critic_optimizer": self.critic_optimizer.state_dict(),
                "log_alpha": self.log_alpha.detach().cpu(),
                "alpha_optimizer": (
                    None
                    if self.alpha_optimizer is None
                    else self.alpha_optimizer.state_dict()
                ),
                "global_environment_step": self.global_environment_step,
                "update_step": self.update_step,
            }
            torch.save(payload, str(path))

    def load_checkpoint(self, path):
        payload = torch.load(str(path), map_location=self.device, weights_only=False)
        if payload.get("version") != "astradrone_sac_checkpoint_v1.0":
            raise ValueError("unsupported SAC checkpoint")
        if payload.get("config") != asdict(self.config):
            raise ValueError("SAC checkpoint config mismatch")
        with self.lock:
            self.actor.load_state_dict(payload["actor"])
            self.critic1.load_state_dict(payload["critic1"])
            self.critic2.load_state_dict(payload["critic2"])
            self.target_critic1.load_state_dict(payload["target_critic1"])
            self.target_critic2.load_state_dict(payload["target_critic2"])
            self.actor_optimizer.load_state_dict(payload["actor_optimizer"])
            self.critic_optimizer.load_state_dict(payload["critic_optimizer"])
            self.log_alpha.data.copy_(payload["log_alpha"].to(self.device))
            if self.alpha_optimizer is not None:
                self.alpha_optimizer.load_state_dict(payload["alpha_optimizer"])
            self.global_environment_step = int(payload["global_environment_step"])
            self.update_step = int(payload["update_step"])
        return payload

    def state_equal(self, other):
        if not isinstance(other, SacAgent):
            return False
        pairs = (
            (self.actor, other.actor),
            (self.critic1, other.critic1),
            (self.critic2, other.critic2),
            (self.target_critic1, other.target_critic1),
            (self.target_critic2, other.target_critic2),
        )
        return bool(
            all(
                all(
                    torch.equal(left.state_dict()[key].cpu(), right.state_dict()[key].cpu())
                    for key in left.state_dict()
                )
                for left, right in pairs
            )
            and torch.equal(self.log_alpha.detach().cpu(), other.log_alpha.detach().cpu())
            and self.global_environment_step == other.global_environment_step
            and self.update_step == other.update_step
        )


class LearnerWorker:
    """Wall-time paced learner isolated from the ROS/sim-time scheduler."""

    def __init__(self, agent, replay, updates_per_second, seed=1):
        self.agent = agent
        self.replay = replay
        self.updates_per_second = float(updates_per_second)
        if self.updates_per_second <= 0.0:
            raise ValueError("updates_per_second must be positive")
        self.rng = np.random.default_rng(int(seed))
        self._condition = threading.Condition()
        self._enabled = False
        self._stop = False
        self._thread = None
        self.metrics = []
        self.failure = ""
        self.started_wall = None
        self.finished_wall = None

    def start(self):
        if self._thread is not None:
            raise RuntimeError("learner worker was already started")
        self._thread = threading.Thread(
            target=self._run, name="astra_sac_learner", daemon=True
        )
        self._thread.start()

    def enable(self):
        with self._condition:
            self._enabled = True
            self._condition.notify_all()

    def stop(self):
        with self._condition:
            self._stop = True
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=30.0)
            if self._thread.is_alive():
                raise RuntimeError("learner worker did not stop")

    def _run(self):
        period = 1.0 / self.updates_per_second
        next_update = time.monotonic()
        try:
            while True:
                with self._condition:
                    while not self._stop and not self._enabled:
                        self._condition.wait(0.1)
                    if self._stop:
                        return
                if len(self.replay) < self.agent.config.batch_size:
                    time.sleep(0.01)
                    continue
                now = time.monotonic()
                if now < next_update:
                    time.sleep(min(next_update - now, 0.02))
                    continue
                batch = self.replay.sample(self.agent.config.batch_size, self.rng)
                started = time.monotonic()
                metric = self.agent.update(batch)
                finished = time.monotonic()
                metric["wall_time"] = time.time()
                metric["update_duration_wall_sec"] = finished - started
                metric["replay_size"] = len(self.replay)
                metric["global_environment_step"] = int(
                    self.agent.global_environment_step
                )
                self.metrics.append(metric)
                if self.started_wall is None:
                    self.started_wall = started
                self.finished_wall = finished
                next_update = max(next_update + period, finished)
        except Exception as error:
            self.failure = "{}: {}".format(type(error).__name__, error)
            with self._condition:
                self._stop = True
