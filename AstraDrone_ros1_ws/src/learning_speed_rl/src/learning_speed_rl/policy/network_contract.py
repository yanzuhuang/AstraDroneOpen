"""Framework-neutral CNN + MLP fusion contract.

No untrained neural network is instantiated in the flight process.  Training
code may implement this contract in PyTorch or another framework, export a
reviewed model, and provide a separate inference backend.
"""

from dataclasses import dataclass
from abc import abstractmethod
from typing import Tuple

from learning_speed_rl.observation.types import LOW_DIM_FIELDS, MAP_CHANNELS
from learning_speed_rl.policy.base import SpeedPolicy


@dataclass(frozen=True)
class FeatureFusionContract:
    contract_version: int = 1
    map_channels: Tuple[str, ...] = MAP_CHANNELS
    low_dim_fields: Tuple[str, ...] = LOW_DIM_FIELDS
    environment_encoder: str = "trajectory_conditioned_cnn"
    state_encoder: str = "mlp"
    fusion: str = "concatenate_then_mlp"
    output_name: str = "raw_v_max_mps"
    output_dimension: int = 1

    def validate(self) -> None:
        if self.contract_version != 1:
            raise ValueError("unsupported feature-fusion contract version")
        if self.map_channels != MAP_CHANNELS or self.low_dim_fields != LOW_DIM_FIELDS:
            raise ValueError("model inputs do not match the deployed observation contract")
        if self.output_dimension != 1:
            raise ValueError("the speed policy must output exactly one scalar")


class FeatureFusionSpeedPolicy(SpeedPolicy):
    """Backend boundary for CNN map encoding + MLP state encoding + fusion."""

    def __init__(self, contract=None):
        self.contract = contract or FeatureFusionContract()
        self.contract.validate()

    def predict(self, observation) -> float:
        if not observation.ready_for_rl:
            raise RuntimeError("feature-fusion inference requires a complete observation")
        environment_feature = self.encode_environment(observation.map_tensor)
        state_feature = self.encode_state(observation.normalized_low_dim)
        return float(self.fuse_and_predict(environment_feature, state_feature))

    @abstractmethod
    def encode_environment(self, map_tensor):
        """CNN or exported-CNN backend."""
        raise NotImplementedError

    @abstractmethod
    def encode_state(self, normalized_low_dim):
        """MLP or exported-MLP backend."""
        raise NotImplementedError

    @abstractmethod
    def fuse_and_predict(self, environment_feature, state_feature):
        """Concatenate features and return one raw velocity limit."""
        raise NotImplementedError
