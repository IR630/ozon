# -*- coding: utf-8 -*-
"""The occlusion probe must drive the real runner and read the real perception line."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from build_probe_items import PROBES  # noqa: E402
from probe_occlusion_heads import (  # noqa: E402
    ANTAGONISTS,
    CONFIGS,
    parse_measurement,
    pose_quat,
    run_cell,
    spawn_pose,
)

_PERCEPTION_LINE = (
    "[python3-2] [INFO] [1784738247.9] [perception]: "
    "item 1: 324x298x284 mm K=0.80 at (1.84, 0.00) heads=3")


def _stub_runner(tmp_path, name, body):
    """A SKELETON stub that runs on ANY host, returned as a `SKELETON` command.

    These stubs used to be bash scripts, and on a Windows host all three seam
    tests failed at 127: `subprocess` there resolves `bash` to the WSL launcher
    in System32, not to Git Bash, and WSL cannot open the Windows path pytest
    hands out -- `C:/Users/...` is not a path inside WSL, in any slash spelling.
    The seam under test is "run this command with this environment", not "run
    this shell", so the stub is Python and the dependency disappears. The real
    runner stays bash; nothing here ever executed it.
    """
    script = tmp_path / name
    script.write_text(body)
    # run_cell splits SKELETON on whitespace, as the production value
    # ("bash scripts/run_skeleton.sh") allows; keep the stub command in that shape.
    assert " " not in sys.executable, f"interpreter path must not contain spaces: {sys.executable}"
    return f"{sys.executable} {script}"


def test_identity_pose_is_the_identity_quaternion():
    assert pose_quat((0.0, 0.0, 1.0), 0) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_a_quarter_turn_about_z_is_a_unit_quaternion_of_the_right_angle():
    x, y, z, w = pose_quat((0.0, 0.0, 1.0), 90)
    assert (x, y) == pytest.approx((0.0, 0.0))
    assert (z, w) == pytest.approx((np.sqrt(0.5), np.sqrt(0.5)))
    assert np.hypot(z, w) == pytest.approx(1.0)


def test_the_measurement_parser_reads_dims_and_head_count():
    dims, k, heads = parse_measurement(_PERCEPTION_LINE)
    assert dims == [324.0, 298.0, 284.0]      # sorted descending
    assert k == pytest.approx(0.80)
    assert heads == 3


def test_the_freshest_measurement_wins_as_in_census_tolerance():
    older = _PERCEPTION_LINE.replace("324x298x284", "999x999x999")
    dims, _k, _heads = parse_measurement(older + "\n" + _PERCEPTION_LINE)
    assert dims == [324.0, 298.0, 284.0]


def test_a_log_without_a_measurement_is_none_not_a_guess():
    assert parse_measurement("helmet -> B: FAIL (pose x=1)\n") is None


@pytest.mark.parametrize("slug,pose_name", ANTAGONISTS)
def test_every_antagonist_pose_exists_on_its_probe(slug, pose_name):
    """A typo here would silently measure a different pose than the one named."""
    assert pose_name in {name for name, _axis, _deg in PROBES[slug].poses}


@pytest.mark.parametrize("slug,pose_name", ANTAGONISTS)
def test_every_antagonist_pose_rests_above_the_belt_not_inside_it(slug, pose_name):
    """A spawn height below the belt buries the body and the solver ejects it."""
    axis, degrees = next((a, d) for name, a, d in PROBES[slug].poses if name == pose_name)
    spawn_z, spawn_y = spawn_pose(slug, pose_quat(axis, degrees))
    assert spawn_z > 0.4                       # BELT_TOP_Z_M
    assert abs(spawn_y) < 0.3                  # stays on the belt, not in the rail


def test_the_two_rigs_differ_only_in_world_and_bridge():
    """If both configs named the same world the 'delta' would be pure run-to-run noise."""
    worlds = {world for _label, world, _bridge in CONFIGS}
    bridges = {bridge for _label, _world, bridge in CONFIGS}
    assert len(worlds) == len(bridges) == len(CONFIGS)


def test_the_cell_passes_the_pose_and_the_rig_to_the_runner(tmp_path, monkeypatch):
    """The seam a stub runner uses; a cell that ignored WORLD would compare nothing."""
    monkeypatch.setenv("SKELETON", _stub_runner(tmp_path, "stub.py", (
        "import os, sys\n"
        "e = os.environ\n"
        "print(f\"world={e['WORLD']} bridge={e['BRIDGE_CONFIG']} "
        "root={e['ITEM_MODEL_ROOT']}\")\n"
        "print(f\"quat={e['ORIENT_X']},{e['ORIENT_Y']},{e['ORIENT_Z']},{e['ORIENT_W']} "
        "z={e['SPAWN_Z']}\")\n"
        "print('args=' + ' '.join(sys.argv[1:]))\n")))
    text = run_cell("ring", "flat", "sim/worlds/cell_diverter_3cam.sdf", "bridge_3cam.yaml",
                    tmp_path)
    assert "world=sim/worlds/cell_diverter_3cam.sdf" in text
    assert "bridge=bridge_3cam.yaml" in text
    assert "root=sim/models/probe_items" in text
    assert f"args=ring {PROBES['ring'].expected}" in text
    assert (tmp_path / "ring_flat_cell_diverter_3cam.log").exists()


def test_the_measurement_is_recovered_from_the_node_log(tmp_path, monkeypatch):
    """run_skeleton echoes only the last three `item N:` lines, often not the
    perception one — the cell must still find the measurement it exists to read."""
    quiet = _stub_runner(tmp_path, "quiet.py", "print('ring -> D: PASS (pose x=4)')\n")
    # The cell's OWN node log, at the path run_cell hands the runner — a real
    # episode writes it there, and reading it back is what this test locks.
    node_log = tmp_path / "ring_flat_cell_diverter.node.log"
    node_log.write_text("[classifier]: item 1: D (k=0.98)\n" + _PERCEPTION_LINE + "\n")
    monkeypatch.setenv("SKELETON", quiet)
    text = run_cell("ring", "flat", "sim/worlds/cell_diverter.sdf", "bridge.yaml", tmp_path)
    dims, _k, heads = parse_measurement(text)
    assert dims == [324.0, 298.0, 284.0]
    assert heads == 3


def test_the_cell_hands_the_runner_its_own_node_log(tmp_path, monkeypatch):
    """The seam itself: whatever NODE_LOG the environment carries, the runner must
    be given the CELL's path. Concurrent runs sharing one file is how a probe cell
    reported another world's measurement as its own."""
    monkeypatch.setenv("SKELETON", _stub_runner(
        tmp_path, "echo_env.py", "import os\nprint('node_log=' + os.environ['NODE_LOG'])\n"))
    monkeypatch.setenv("NODE_LOG", "/tmp/skeleton_e2e.log")
    text = run_cell("ring", "flat", "sim/worlds/cell_diverter.sdf", "bridge.yaml", tmp_path)
    assert f"node_log={tmp_path / 'ring_flat_cell_diverter.node.log'}" in text


def test_a_wedged_episode_is_recorded_instead_of_crashing_the_sweep(tmp_path, monkeypatch):
    monkeypatch.setenv("SKELETON", _stub_runner(
        tmp_path, "hang.py", "import time\ntime.sleep(30)\n"))
    # No node log is written for this cell, so nothing may be recovered. Before
    # the per-cell NODE_LOG this read the machine-wide /tmp/skeleton_e2e.log and
    # a parallel census made the wedged cell "recover" a measurement it never
    # took — 401x400x302 heads=1, from another world.
    monkeypatch.setenv("NODE_LOG", "/tmp/skeleton_e2e.log")
    text = run_cell("ring", "flat", "sim/worlds/cell_diverter.sdf", "bridge.yaml",
                    tmp_path, timeout_s=1)
    assert "TIMEOUT" in text
    assert parse_measurement(text) is None
