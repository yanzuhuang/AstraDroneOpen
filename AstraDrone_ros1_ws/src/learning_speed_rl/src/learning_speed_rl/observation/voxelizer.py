"""Fixed-size crop for the currently exported occupied-point geometry."""

from typing import Iterable, Optional, Sequence

import numpy as np

from .types import MapTensorSpec, vector3


class OccupiedPointCloudVoxelizer:
    """Build the v1 tensor without inventing unavailable free-space evidence.

    The current EGO PointCloud2 export contains inflated occupied voxels only.
    Therefore all unobserved cells remain `unknown`, and the tensor must keep
    `map_semantics_complete=False` until a categorical map export is added.
    """

    def __init__(self, spec: MapTensorSpec):
        self.spec = spec

    def build(
        self,
        occupied_points_xyz: Iterable[Sequence[float]],
        center_xyz: Sequence[float],
        trajectory_points_xyz: Optional[Iterable[Sequence[float]]] = None,
    ) -> np.ndarray:
        center = vector3(center_xyz, "center_xyz").astype(np.float64)
        tensor = np.zeros(self.spec.tensor_shape, dtype=np.float32)
        tensor[2, ...] = 1.0  # Unknown until a map source proves otherwise.
        lower = center - np.asarray(self.spec.size_xyz_m, dtype=np.float64) / 2.0

        self._mark_points(tensor, occupied_points_xyz, lower, channel=1)
        occupied = tensor[1, ...] > 0.5
        tensor[2, occupied] = 0.0

        if trajectory_points_xyz is not None:
            self._mark_points(tensor, trajectory_points_xyz, lower, channel=3)
        return tensor

    def _mark_points(self, tensor, points, lower, channel):
        xyz_shape = (self.spec.shape_zyx[2], self.spec.shape_zyx[1], self.spec.shape_zyx[0])
        for point in points:
            values = np.asarray(point, dtype=np.float64)
            if values.shape != (3,) or not np.all(np.isfinite(values)):
                continue
            index_xyz = np.floor((values - lower) / self.spec.resolution_m).astype(np.int64)
            if np.any(index_xyz < 0) or any(index_xyz[i] >= xyz_shape[i] for i in range(3)):
                continue
            tensor[channel, index_xyz[2], index_xyz[1], index_xyz[0]] = 1.0
