"""Abstract training environment boundary and artifact-location guard."""

from abc import ABC, abstractmethod
from pathlib import Path


class SpeedTrainingEnvironment(ABC):
    @abstractmethod
    def reset(self):
        raise NotImplementedError

    @abstractmethod
    def run_episode(self, action_provider, config=None):
        raise NotImplementedError


def validate_artifact_root(path: str) -> Path:
    resolved = Path(path).expanduser().resolve()
    repository_root = Path(__file__).resolve().parents[6]
    allowed_root = (repository_root / "runtime_artifacts" / "learning_speed").resolve()
    if resolved != allowed_root and allowed_root not in resolved.parents:
        raise ValueError(
            "training output must be under {}".format(allowed_root)
        )
    return resolved
