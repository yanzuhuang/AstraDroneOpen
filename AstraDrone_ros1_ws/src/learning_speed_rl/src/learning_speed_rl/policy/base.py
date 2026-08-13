"""Policy API used identically by mock and future trained inference."""

from abc import ABC, abstractmethod

from learning_speed_rl.observation.types import PolicyObservation


class SpeedPolicy(ABC):
    @abstractmethod
    def predict(self, observation: PolicyObservation) -> float:
        """Return a raw requested maximum velocity in metres per second."""
        raise NotImplementedError
