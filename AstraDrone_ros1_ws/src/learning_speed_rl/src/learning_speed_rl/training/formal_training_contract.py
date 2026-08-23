"""Pure mode and Episode schedule contract for formal SAC training/evaluation."""

from dataclasses import dataclass


SUPPORTED_RUNNER_MODES = ("qualification", "training", "evaluation")


def mode_uses_training_replay(mode):
    if mode not in SUPPORTED_RUNNER_MODES:
        raise ValueError("unsupported SAC runner mode")
    return mode != "evaluation"


def learning_started(valid_transitions, learning_starts):
    """Keep the learner gate explicitly transition-based."""

    transitions = int(valid_transitions)
    threshold = int(learning_starts)
    if transitions < 0 or threshold <= 0:
        raise ValueError("transition learning-start counts are invalid")
    return transitions >= threshold


def checkpoint_filename(completed_episode):
    """Return the unambiguous Episode-based formal checkpoint name."""

    episode = int(completed_episode)
    if episode <= 0:
        raise ValueError("checkpoint Episode must be positive")
    return "sac_checkpoint_episode_{:04d}.pt".format(episode)


@dataclass(frozen=True)
class FormalTrainingSchedule:
    total_training_episodes: int
    checkpoint_episodes: tuple

    def validate(self):
        target = int(self.total_training_episodes)
        episodes = tuple(
            sorted(set(int(value) for value in self.checkpoint_episodes))
        )
        if target <= 0:
            raise ValueError("total training Episodes must be positive")
        if not episodes or any(value <= 0 or value > target for value in episodes):
            raise ValueError(
                "checkpoint Episodes must be inside the training Episode target"
            )
        if episodes[-1] != target:
            raise ValueError(
                "final training Episode must be an explicit checkpoint"
            )

    def checkpoint_due(self, completed_episodes):
        self.validate()
        return int(completed_episodes) in self.checkpoint_episodes

    def next_episode_number(self, completed_episodes):
        """Return the only legal next Episode number."""

        self.validate()
        completed = int(completed_episodes)
        if completed < 0 or completed >= self.total_training_episodes:
            raise ValueError("training has no remaining Episode budget")
        return completed + 1

    def complete_episode(self, completed_episodes, terminal_transition_closed):
        """Count exactly one Episode only after its terminal transition closes."""

        if not bool(terminal_transition_closed):
            raise ValueError("an unclosed Episode cannot count as completed")
        return self.next_episode_number(completed_episodes)

    def stop_due(self, completed_episodes):
        self.validate()
        value = int(completed_episodes)
        if value < 0 or value > self.total_training_episodes:
            raise ValueError("training exceeded the completed Episode target")
        return value == self.total_training_episodes
