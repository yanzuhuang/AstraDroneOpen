"""Pure identity and generation gates for training Episode/reset integration."""

from dataclasses import dataclass


@dataclass(frozen=True)
class EpisodeBinding:
    episode_id: int
    reset_generation: int

    @property
    def episode_key(self):
        return "training_episode_{:06d}".format(self.episode_id)

    def validate(self):
        if self.episode_id <= 0:
            raise ValueError("episode_id must be positive")
        if self.reset_generation < 0:
            raise ValueError("reset_generation must be non-negative")


class EpisodeIdentityLedger:
    """Own the exact one-reset/one-generation/one-next-episode relation."""

    def __init__(self):
        self._current = EpisodeBinding(episode_id=1, reset_generation=0)

    @property
    def current(self):
        return self._current

    def advance_after_reset(self, observed_generation):
        expected = self._current.reset_generation + 1
        if int(observed_generation) != expected:
            raise ValueError(
                "reset generation must advance exactly once: expected {}, got {}"
                .format(expected, observed_generation)
            )
        self._current = EpisodeBinding(
            episode_id=self._current.episode_id + 1,
            reset_generation=expected,
        )
        return self._current


def action_matches(binding, episode_key, step_index, request_id):
    binding.validate()
    return bool(
        str(episode_key) == binding.episode_key
        and int(step_index) == 0
        and int(request_id) == binding.episode_id
    )


def sac_closure_matches(binding, payload):
    """Require exact terminal-transition closure before coordinator reset."""

    binding.validate()
    if not isinstance(payload, dict):
        return False
    try:
        return bool(
            str(payload.get("episode_id", "")) == binding.episode_key
            and int(payload.get("reset_generation", -1))
            == binding.reset_generation
            and bool(payload.get("terminal_transition_closed", False))
            and int(payload.get("transition_count", 0)) > 0
            and int(payload.get("last_step_index", -1)) >= 0
            and int(payload.get("last_request_id", 0))
            == int(payload.get("last_step_index", -1)) + 1
            and str(payload.get("status", "")) == "completed"
        )
    except (TypeError, ValueError):
        return False


def trajectory_matches(
    binding,
    trajectory_id,
    trajectory_start_sec,
    callback_sequence,
    goal_sequence,
    goal_stamp_sec,
    reset_barrier_sec,
    previous_trajectory_id,
):
    binding.validate()
    return bool(
        int(trajectory_id) > int(previous_trajectory_id)
        and int(callback_sequence) > int(goal_sequence)
        and float(trajectory_start_sec) > float(reset_barrier_sec)
        and float(trajectory_start_sec) + 1.0e-9 >= float(goal_stamp_sec)
    )


def observation_matches(
    binding,
    valid,
    temporal_generation,
    lidar_temporal_generation,
    source_stamp_sec,
    reset_barrier_sec,
    trajectory_id,
    accepted_trajectory_id,
):
    binding.validate()
    return bool(
        valid
        and int(temporal_generation) == binding.reset_generation
        and int(lidar_temporal_generation) == binding.reset_generation
        and float(source_stamp_sec) > float(reset_barrier_sec)
        and int(trajectory_id) >= int(accepted_trajectory_id)
    )
