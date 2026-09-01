"""Pure contracts for qualified Forest world loading and switch requests."""

from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


FOREST_PREFIXES = ("sparse_", "medium_", "dense_")


@dataclass(frozen=True)
class ForestModelSpec:
    name: str
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    radius: float
    height: float
    model_sdf: str


class ForestPoolContract:
    def __init__(self, config_path, world_dir):
        self.config_path = Path(config_path).resolve()
        self.world_dir = Path(world_dir).resolve()
        self.config = json.loads(self.config_path.read_text(encoding="utf-8"))
        self.seed_pool = {
            int(item["logical_seed"]): dict(item)
            for item in self.config["seed_pool"]
        }
        if set(self.seed_pool) != set(range(10)):
            raise ValueError("Forest pool must define logical seed0-9")
        if {seed for seed, item in self.seed_pool.items() if item["role"] == "training"} != set(range(8)):
            raise ValueError("Forest training pool must be logical seed0-7")
        if {seed for seed, item in self.seed_pool.items() if item["role"] == "evaluation"} != {8, 9}:
            raise ValueError("Forest evaluation pool must be logical seed8-9")
        self.layouts = {
            seed: self._parse_world(
                self.world_dir / "learning_speed_forest_v1_seed{}.world".format(seed)
            )
            for seed in range(10)
        }

    @staticmethod
    def _parse_world(path):
        world = ET.parse(path).getroot().find("world")
        if world is None:
            raise ValueError("Forest world has no <world>: {}".format(path))
        result = []
        for model in world.findall("model"):
            cylinder = model.find("./link/collision/geometry/cylinder")
            if cylinder is None:
                continue
            name = str(model.get("name", ""))
            if not name.startswith(FOREST_PREFIXES):
                raise ValueError("unexpected Forest model name: " + name)
            pose_values = [float(value) for value in model.findtext("pose", "").split()]
            if len(pose_values) != 6:
                raise ValueError("Forest model pose must contain six values")
            model_copy = deepcopy(model)
            pose = model_copy.find("pose")
            if pose is not None:
                model_copy.remove(pose)
            sdf = ET.Element("sdf", {"version": "1.7"})
            sdf.append(model_copy)
            result.append(
                ForestModelSpec(
                    name=name,
                    x=pose_values[0], y=pose_values[1], z=pose_values[2],
                    roll=pose_values[3], pitch=pose_values[4], yaw=pose_values[5],
                    radius=float(cylinder.findtext("radius", "nan")),
                    height=float(cylinder.findtext("length", "nan")),
                    model_sdf=ET.tostring(sdf, encoding="unicode"),
                )
            )
        result.sort(key=lambda item: int(item.name.split("_")[-1]))
        if len(result) != 18 or len({item.name for item in result}) != 18:
            raise ValueError("Forest world must contain 18 unique cylinders")
        return tuple(result)

    def mapping(self):
        return {seed: int(item["raw_seed"]) for seed, item in self.seed_pool.items()}

    def role(self, logical_seed):
        seed = int(logical_seed)
        if seed not in self.seed_pool:
            raise ValueError("logical Forest seed is outside seed0-9")
        return str(self.seed_pool[seed]["role"])

    def validate_request(self, payload):
        required = {
            "request_id", "logical_seed", "raw_seed", "mode",
            "map_block_id", "map_round_id", "scheduler_order",
        }
        if set(payload) != required:
            raise ValueError("Forest map request fields do not match contract")
        request_id = str(payload["request_id"])
        logical_seed = int(payload["logical_seed"])
        raw_seed = int(payload["raw_seed"])
        mode = str(payload["mode"])
        if not request_id:
            raise ValueError("Forest map request_id is empty")
        if raw_seed != self.mapping()[logical_seed]:
            raise ValueError("logical/raw Forest seed mapping mismatch")
        if mode not in ("training", "evaluation") or self.role(logical_seed) != mode:
            raise ValueError("Forest map mode violates training/evaluation isolation")
        order = tuple(int(value) for value in payload["scheduler_order"])
        if mode == "training" and set(order) != set(range(8)):
            raise ValueError("training scheduler order must contain seed0-7 exactly")
        if mode == "evaluation" and order != (logical_seed,):
            raise ValueError("evaluation request must contain only its unseen seed")
        result = dict(payload)
        result.update(
            request_id=request_id,
            logical_seed=logical_seed,
            raw_seed=raw_seed,
            mode=mode,
            map_block_id=int(payload["map_block_id"]),
            map_round_id=int(payload["map_round_id"]),
            scheduler_order=list(order),
        )
        return result

    def layout_hash(self, logical_seed):
        records = self.layouts[int(logical_seed)]
        value = "\n".join(
            "{}|{:.9f}|{:.9f}|{:.9f}|{:.9f}|{:.9f}".format(
                item.name, item.x, item.y, item.z, item.radius, item.height
            )
            for item in records
        )
        return hashlib.sha256(value.encode("utf-8")).hexdigest()

    @staticmethod
    def verify_loaded_layout(expected, world_model_names, model_positions, tolerance=1.0e-4):
        expected_names = {item.name for item in expected}
        forest_names = {name for name in world_model_names if str(name).startswith(FOREST_PREFIXES)}
        failures = []
        if forest_names != expected_names:
            failures.append("forest_model_set_mismatch")
        for item in expected:
            position = model_positions.get(item.name)
            if position is None:
                failures.append("missing_model_state:" + item.name)
                continue
            if max(abs(float(actual) - expected_value) for actual, expected_value in zip(position, (item.x, item.y, item.z))) > tolerance:
                failures.append("model_pose_mismatch:" + item.name)
        return {
            "passed": not failures,
            "failures": failures,
            "expected_obstacle_count": len(expected_names),
            "loaded_obstacle_count": len(forest_names),
            "residual_obstacle_count": len(forest_names - expected_names),
        }
