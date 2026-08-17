"""Source-time helpers that deliberately avoid ROS float round trips."""


def preserve_source_stamp(source_stamp):
    """Return the exact source stamp object, retaining its sec/nsec pair."""
    return source_stamp
