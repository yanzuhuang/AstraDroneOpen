"""Fail-closed model selection contract for future RL inference."""

from pathlib import Path

from learning_speed_rl.policy.base import SpeedPolicy


class ReviewedModelRunner(SpeedPolicy):
    def __init__(self, model_path: str, backend):
        path = Path(model_path).expanduser().resolve()
        if not path.is_file():
            raise ValueError("reviewed model file does not exist: {}".format(path))
        if backend is None:
            raise ValueError("an explicit reviewed inference backend is required")
        self.model_path = path
        self.backend = backend

    def predict(self, observation) -> float:
        if not observation.ready_for_rl:
            raise RuntimeError("RL inference blocked: observation contract is incomplete")
        return float(self.backend.predict(observation))
