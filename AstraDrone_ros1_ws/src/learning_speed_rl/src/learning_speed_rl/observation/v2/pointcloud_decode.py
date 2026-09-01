"""Vectorized PointCloud2 XYZ decoding with the ROS field layout preserved."""

import numpy as np


_FLOAT_FORMATS = {
    7: "f4",  # sensor_msgs/PointField.FLOAT32
    8: "f8",  # sensor_msgs/PointField.FLOAT64
}


def decode_xyz_points(message, maximum_input_points):
    """Return the first finite XYZ points in PointCloud2 row-major order.

    The input limit is applied before finite filtering, matching the former
    ``read_points`` loop.  Organized clouds and row padding are handled through
    explicit NumPy strides.
    """

    limit = int(maximum_input_points)
    if limit <= 0:
        raise ValueError("maximum_input_points must be positive")
    fields = {str(field.name): field for field in message.fields}
    if any(name not in fields for name in ("x", "y", "z")):
        raise KeyError("PointCloud2 is missing x/y/z")
    endian = ">" if bool(message.is_bigendian) else "<"
    formats = []
    offsets = []
    for name in ("x", "y", "z"):
        field = fields[name]
        if int(field.count) != 1 or int(field.datatype) not in _FLOAT_FORMATS:
            raise TypeError("PointCloud2 x/y/z must be scalar float fields")
        formats.append(endian + _FLOAT_FORMATS[int(field.datatype)])
        offsets.append(int(field.offset))
    point_step = int(message.point_step)
    width = int(message.width)
    height = int(message.height)
    row_step = int(message.row_step)
    if min(point_step, width, height, row_step) <= 0:
        return np.empty((0, 3), dtype=np.float32)
    dtype = np.dtype(
        {
            "names": ("x", "y", "z"),
            "formats": formats,
            "offsets": offsets,
            "itemsize": point_step,
        }
    )
    required = (height - 1) * row_step + width * point_step
    if required > len(message.data):
        raise ValueError("PointCloud2 data is shorter than its declared layout")
    structured = np.ndarray(
        shape=(height, width),
        dtype=dtype,
        buffer=message.data,
        strides=(row_step, point_step),
    ).reshape(-1)[:limit]
    if structured.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    points = np.column_stack(
        (structured["x"], structured["y"], structured["z"])
    ).astype(np.float32, copy=False)
    return points[np.all(np.isfinite(points), axis=1)]
