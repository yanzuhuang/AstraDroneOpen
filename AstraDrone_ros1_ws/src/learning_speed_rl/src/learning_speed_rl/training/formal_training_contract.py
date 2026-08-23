"""Pure mode and step schedule contract for formal SAC training/evaluation."""

from dataclasses import dataclass


SUPPORTED_RUNNER_MODES = ("qualification", "training", "evaluation")


def mode_uses_training_replay(mode):
    if mode not in SUPPORTED_RUNNER_MODES:
        raise ValueError("unsupported SAC runner mode")
    return mode != "evaluation"


def mode_updates_networks(mode):
    if mode not in SUPPORTED_RUNNER_MODES:
        raise ValueError("unsupported SAC runner mode")
    return mode != "evaluation"


@dataclass(frozen=True)
class FormalTrainingSchedule:
    target_valid_transitions: int
    checkpoint_steps: tuple
    checkpoint_interval: int
    evaluation_during_training: bool

    def validate(self):
        target = int(self.target_valid_transitions)
        steps = tuple(sorted(set(int(value) for value in self.checkpoint_steps)))
        if target <= 0 or self.checkpoint_interval <= 0:
            raise ValueError("training target/checkpoint interval must be positive")
        if not steps or any(value <= 0 or value > target for value in steps):
            raise ValueError("checkpoint steps must be inside the training target")
        if steps[-1] != target:
            raise ValueError("final target must be an explicit checkpoint step")
        if any(value % self.checkpoint_interval != 0 for value in steps):
            raise ValueError("checkpoint steps disagree with checkpoint interval")
        if self.evaluation_during_training:
            raise ValueError("formal training must not run evaluation")

    def checkpoint_due(self, environment_step):
        self.validate()
        return int(environment_step) in self.checkpoint_steps

    def next_transition_step(self, completed_transitions):
        """Return the only legal next Replay/environment step."""

        self.validate()
        completed = int(completed_transitions)
        if completed < 0 or completed >= self.target_valid_transitions:
            raise ValueError("training has no remaining valid transition budget")
        return completed + 1

    def episode_step_budget(self, completed_transitions, configured_max_steps):
        """Cap one Episode so in-flight work cannot cross the formal target."""

        self.validate()
        completed = int(completed_transitions)
        if completed < 0 or completed >= self.target_valid_transitions:
            raise ValueError("training has no remaining valid transition budget")
        remaining = self.target_valid_transitions - completed
        if configured_max_steps is None:
            return remaining
        configured = int(configured_max_steps)
        if configured <= 0:
            raise ValueError("configured Episode step limit must be positive")
        return min(remaining, configured)

    def stop_due(self, environment_step):
        self.validate()
        value = int(environment_step)
        if value > self.target_valid_transitions:
            raise ValueError("training exceeded target valid transitions")
        return value == self.target_valid_transitions
