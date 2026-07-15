#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Extract requested model poses from one Ignition dynamic-pose JSON message.

Input is produced by:
    ign topic -e --json-output -n 1 -t /world/cell/dynamic_pose/info

One output row per found model:
    name x y z roll pitch yaw
"""
import json
import math
import sys


def quaternion_to_rpy(orientation):
    """Ignition quaternion mapping (missing JSON fields mean protobuf zero)."""
    x = float(orientation.get("x", 0.0))
    y = float(orientation.get("y", 0.0))
    z = float(orientation.get("z", 0.0))
    w = float(orientation.get("w", 0.0))

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def requested_poses(message, names):
    """Return requested model poses in request order; absent models are omitted."""
    by_name = {}
    for pose in message.get("pose", []):
        name = pose.get("name")
        if name in names and name not in by_name:
            position = pose.get("position", {})
            x = float(position.get("x", 0.0))
            y = float(position.get("y", 0.0))
            z = float(position.get("z", 0.0))
            by_name[name] = (x, y, z, *quaternion_to_rpy(pose.get("orientation", {})))
    return [(name, *by_name[name]) for name in names if name in by_name]


def main(argv=None):
    names = sys.argv[1:] if argv is None else argv
    if not names:
        raise SystemExit("usage: pose_snapshot.py <model_name> ...")
    try:
        # Fortress occasionally prints two complete one-line JSON messages even
        # with `-n 1`.  Use the newest valid snapshot instead of rejecting the
        # whole poll as "Extra data".
        messages = [json.loads(line) for line in sys.stdin if line.strip()]
        message = messages[-1]
    except (json.JSONDecodeError, OSError, IndexError) as exc:
        raise SystemExit(f"invalid dynamic-pose JSON: {exc}") from None
    for row in requested_poses(message, names):
        print(" ".join([row[0], *(f"{value:.9f}" for value in row[1:])]))


if __name__ == "__main__":
    main()
