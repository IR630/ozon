# -*- coding: utf-8 -*-
"""End-to-end zone routing (day 5, P4): one item of each category rides the
FULL contour (Gazebo + camera -> perception -> classifier -> controller ->
actuator) into its zone, as a pytest — the same check run_skeleton.sh does by
hand, wired into the suite so a broken skeleton fails a test run, not a demo.

Needs the whole runtime (Gazebo, ros_gz, built ros_msgs overlay) and ~1 min per
episode, so it is opt-in twice: the `slow` marker AND the RUN_E2E=1 env gate —
a plain `pytest -q` anywhere (laptop, CI unit job) stays fast and green.
Inside the Docker/WSL environment:
    RUN_E2E=1 python3 -m pytest -q -m slow tests/test_e2e_zones.py
Mechanism seam carries over: WORLD=sim/worlds/cell_diverter.sdf RUN_E2E=1 ...
"""
import os
import shutil
import subprocess
from pathlib import Path

import pytest

pytestmark = [
    pytest.mark.slow,
    pytest.mark.skipif(not os.environ.get("RUN_E2E"),
                       reason="e2e episodes are opt-in: set RUN_E2E=1"),
    pytest.mark.skipif(shutil.which("ign") is None,
                       reason="needs the Gazebo runtime (Docker/WSL)"),
]

REPO = Path(__file__).resolve().parents[1]
# one item per category, the day-3 skeleton trio; zones per docs/md/models.md
EPISODES = [("box_300x200x200", "B"), ("box_400x400x300", "C"), ("plate", "D")]


@pytest.mark.parametrize("slug,zone", EPISODES)
def test_item_reaches_its_zone(slug, zone):
    proc = subprocess.run(
        ["bash", "scripts/run_skeleton.sh", slug, zone],
        cwd=REPO, capture_output=True, text=True, timeout=180)
    tail = "\n".join(proc.stdout.splitlines()[-5:])
    assert proc.returncode == 0, f"{slug} -> {zone} did not reach its zone:\n{tail}"
