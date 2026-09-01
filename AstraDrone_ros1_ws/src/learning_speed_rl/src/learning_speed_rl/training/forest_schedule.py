"""Deterministic Forest map scheduling, isolated from SAC ownership."""

from dataclasses import dataclass
import random


def _as_tuple(value):
    if isinstance(value, list):
        return tuple(_as_tuple(item) for item in value)
    return value


def _as_list(value):
    if isinstance(value, tuple):
        return [_as_list(item) for item in value]
    return value


@dataclass(frozen=True)
class ForestMapAssignment:
    logical_seed: int
    raw_seed: int
    mode: str
    episode_index: int
    map_block_id: int
    map_round_id: int
    scheduler_order: tuple

    def as_dict(self):
        return {
            "forest_seed": self.logical_seed,
            "raw_seed": self.raw_seed,
            "train_or_evaluation": self.mode,
            "episode_index": self.episode_index,
            "map_block_id": self.map_block_id,
            "map_round_id": self.map_round_id,
            "scheduler_order": list(self.scheduler_order),
        }


class BalancedForestMapScheduler:
    """Balanced map rounds with an explicit bounded-smoke block override."""

    def __init__(
        self,
        training_logical_seeds,
        evaluation_logical_seeds,
        logical_to_raw_seed,
        episodes_per_map_block=100,
        rng_seed=20260824,
        smoke_test=False,
    ):
        self.training_logical_seeds = tuple(int(value) for value in training_logical_seeds)
        self.evaluation_logical_seeds = tuple(int(value) for value in evaluation_logical_seeds)
        self.logical_to_raw_seed = {
            int(key): int(value) for key, value in logical_to_raw_seed.items()
        }
        self.episodes_per_map_block = int(episodes_per_map_block)
        self.rng_seed = int(rng_seed)
        self.smoke_test = bool(smoke_test)
        if self.training_logical_seeds != tuple(range(8)):
            raise ValueError("formal Forest training pool must be logical seed0-7")
        if self.evaluation_logical_seeds != (8, 9):
            raise ValueError("formal Forest evaluation pool must be logical seed8-9")
        if set(self.training_logical_seeds) & set(self.evaluation_logical_seeds):
            raise ValueError("training and evaluation Forest pools overlap")
        if set(self.logical_to_raw_seed) != set(range(10)):
            raise ValueError("logical/raw Forest mapping must cover seed0-9 exactly")
        if self.smoke_test:
            if self.episodes_per_map_block != 10:
                raise ValueError("bounded Forest smoke map block must contain exactly 10 Episodes")
        elif self.episodes_per_map_block != 100:
            raise ValueError("formal Forest map block must contain exactly 100 Episodes")
        self._rng = random.Random(self.rng_seed)
        self._orders = []

    @property
    def episodes_per_round(self):
        return len(self.training_logical_seeds) * self.episodes_per_map_block

    def _ensure_round(self, round_id):
        value = int(round_id)
        if value < 0:
            raise ValueError("map round id must be non-negative")
        while len(self._orders) <= value:
            order = list(self.training_logical_seeds)
            self._rng.shuffle(order)
            self._orders.append(tuple(order))

    def training_assignment(self, episode_index):
        episode = int(episode_index)
        if episode < 0:
            raise ValueError("training Episode index must be non-negative")
        block = episode // self.episodes_per_map_block
        round_id = block // len(self.training_logical_seeds)
        position = block % len(self.training_logical_seeds)
        self._ensure_round(round_id)
        order = self._orders[round_id]
        logical_seed = order[position]
        if logical_seed in self.evaluation_logical_seeds:
            raise RuntimeError("evaluation map entered the training scheduler")
        return ForestMapAssignment(
            logical_seed=logical_seed,
            raw_seed=self.logical_to_raw_seed[logical_seed],
            mode="training",
            episode_index=episode,
            map_block_id=block,
            map_round_id=round_id,
            scheduler_order=order,
        )

    def evaluation_assignment(self, logical_seed, evaluation_episode_index=0):
        logical = int(logical_seed)
        episode = int(evaluation_episode_index)
        if logical not in self.evaluation_logical_seeds:
            raise ValueError("evaluation is restricted to logical seed8 or seed9")
        if episode < 0:
            raise ValueError("evaluation Episode index must be non-negative")
        return ForestMapAssignment(
            logical_seed=logical,
            raw_seed=self.logical_to_raw_seed[logical],
            mode="evaluation",
            episode_index=episode,
            map_block_id=-1,
            map_round_id=-1,
            scheduler_order=(logical,),
        )

    def map_switch_due_after(self, completed_episodes, total_episodes):
        completed = int(completed_episodes)
        total = int(total_episodes)
        if completed < 0 or total <= 0 or completed > total:
            raise ValueError("completed/total Episode counts are invalid")
        return bool(
            completed > 0
            and completed < total
            and completed % self.episodes_per_map_block == 0
        )

    def snapshot(self, completed_episodes):
        completed = int(completed_episodes)
        if completed < 0:
            raise ValueError("completed Episode count must be non-negative")
        if completed:
            self.training_assignment(completed - 1)
        return {
            "version": "astradrone_forest_scheduler_state_v1.0",
            "completed_training_episodes": completed,
            "training_logical_seeds": list(self.training_logical_seeds),
            "evaluation_logical_seeds": list(self.evaluation_logical_seeds),
            "logical_to_raw_seed": {
                str(key): value for key, value in sorted(self.logical_to_raw_seed.items())
            },
            "episodes_per_map_block": self.episodes_per_map_block,
            "episodes_per_round": self.episodes_per_round,
            "rng_seed": self.rng_seed,
            "smoke_test": self.smoke_test,
            "generated_round_orders": [list(order) for order in self._orders],
            "rng_state": _as_list(self._rng.getstate()),
        }

    @classmethod
    def from_snapshot(cls, snapshot):
        if snapshot.get("version") != "astradrone_forest_scheduler_state_v1.0":
            raise ValueError("unsupported Forest scheduler snapshot")
        scheduler = cls(
            snapshot["training_logical_seeds"],
            snapshot["evaluation_logical_seeds"],
            snapshot["logical_to_raw_seed"],
            episodes_per_map_block=snapshot["episodes_per_map_block"],
            rng_seed=snapshot["rng_seed"],
            smoke_test=bool(snapshot.get("smoke_test", False)),
        )
        scheduler._orders = [tuple(int(value) for value in order) for order in snapshot["generated_round_orders"]]
        if any(set(order) != set(scheduler.training_logical_seeds) for order in scheduler._orders):
            raise ValueError("Forest scheduler snapshot has invalid round order")
        scheduler._rng.setstate(_as_tuple(snapshot["rng_state"]))
        completed = int(snapshot["completed_training_episodes"])
        if completed and scheduler.training_assignment(completed - 1).episode_index != completed - 1:
            raise ValueError("Forest scheduler snapshot completion mismatch")
        return scheduler
