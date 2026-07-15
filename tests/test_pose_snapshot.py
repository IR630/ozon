# -*- coding: utf-8 -*-
"""Batch dynamic-pose JSON parsing used by the stream verdict poll."""
import json
import math
import subprocess
import sys
from pathlib import Path

import pytest

from pose_snapshot import quaternion_to_rpy, requested_poses


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "pose_snapshot.py"


def test_requested_models_are_returned_in_roster_order_with_proto_defaults():
    message = {
        "pose": [
            {"name": "belt", "position": {"x": 1}},
            {"name": "item1", "position": {"x": 2.5, "z": 0.4},
             "orientation": {"w": 1}},
            {"name": "item0", "position": {"x": 1.2, "y": -0.3},
             "orientation": {"z": math.sqrt(0.5), "w": math.sqrt(0.5)}},
        ]
    }

    rows = requested_poses(message, ["item0", "item1", "item2"])

    assert [row[0] for row in rows] == ["item0", "item1"]
    assert rows[0][1:4] == pytest.approx((1.2, -0.3, 0.0))
    assert rows[0][4:] == pytest.approx((0.0, 0.0, math.pi / 2.0))
    assert rows[1][1:4] == pytest.approx((2.5, 0.0, 0.4))


def test_quaternion_conversion_handles_roll():
    assert quaternion_to_rpy({"x": math.sqrt(0.5), "w": math.sqrt(0.5)}) == pytest.approx(
        (math.pi / 2.0, 0.0, 0.0), abs=1e-7)


def test_cli_prints_one_machine_readable_row_per_found_item():
    message = {"pose": [{"name": "item0", "position": {"x": 1.25},
                          "orientation": {"w": 1}}]}
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "item0", "item1"],
        input=json.dumps(message), capture_output=True, text=True, check=False,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.split() == ["item0", "1.250000000", "0.000000000",
                                     "0.000000000", "0.000000000", "0.000000000",
                                     "0.000000000"]


def test_cli_uses_latest_snapshot_if_ignition_prints_two_messages():
    old = {"pose": [{"name": "item0", "position": {"x": 1.0}}]}
    new = {"pose": [{"name": "item0", "position": {"x": 2.0}}]}
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "item0"],
        input=f"{json.dumps(old)}\n{json.dumps(new)}\n",
        capture_output=True, text=True, check=False,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.split()[1] == "2.000000000"
