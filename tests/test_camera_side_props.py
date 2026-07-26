# -*- coding: utf-8 -*-
"""The demo props must not become an item the cell tries to sort.

A visual prop that lands in the top head's item mask is not a cosmetic problem:
`_find_items` would return it as a body, the tracker would give it an id, and the
footage would show the cell routing its own tripod. The gantry prop already
carries that argument in its header; these tests hold the SIDE props to it, and
hold both to the poses the constants declare — a prop drawn somewhere other than
the sensor is a picture that lies about the rig.
"""
import re
from pathlib import Path

import pytest

from src.constants import CAMERA_SIDE_NEG_Y_POSE_M, CAMERA_SIDE_POS_Y_POSE_M
from src.perception import (
    BELT_DEPTH_M,
    CAMERA_Z_M,
    FX,
    IMG_W,
    MASK_MARGIN_M,
)

ROOT = Path(__file__).resolve().parents[1]
PROPS = ROOT / "sim" / "models" / "camera_side_props" / "model.sdf"
POST_HALF_W_M = 0.04


def _visuals(text):
    """{name: (x, y, z)} of every <visual> in the model."""
    out = {}
    for name, pose in re.findall(
            r'<visual name="([^"]+)">\s*\n\s*<pose>([^<]+)</pose>', text):
        out[name] = tuple(float(v) for v in pose.split()[:3])
    return out


def _top_half_view_y_m(z_m):
    """Half-width of the top head's view in y at height z (image x maps to world y)."""
    return (IMG_W / 2.0) / FX * (CAMERA_Z_M - z_m)


def _markup(text):
    """The SDF with XML comments stripped — the header discusses <collision>."""
    return re.sub(r"<!--.*?-->", "", text, flags=re.S)


def test_the_props_carry_no_collision_at_all():
    """Physics must be untouched, or the demo world stops being the measured one."""
    markup = _markup(PROPS.read_text(encoding="utf-8"))
    assert "<collision" not in markup
    assert "<static>true</static>" in markup


def test_each_prop_stands_at_the_head_pose_the_constants_declare():
    """A prop drawn away from its sensor turns the footage into an illustration."""
    visuals = _visuals(PROPS.read_text(encoding="utf-8"))
    for key, pose in (("neg_y", CAMERA_SIDE_NEG_Y_POSE_M),
                      ("pos_y", CAMERA_SIDE_POS_Y_POSE_M)):
        x, y, z = pose[0]
        # Housing at the sensor's own height, lens inboard of it, mount behind
        # it — the order a real bracket has, and the order that keeps the mount
        # out of the top head's mask (see the test below).
        assert visuals[f"housing_{key}"][0] == pytest.approx(x, abs=1e-6)
        assert visuals[f"housing_{key}"][2] == pytest.approx(z, abs=1e-6)
        assert abs(visuals[f"lens_{key}"][1]) < abs(y) < abs(visuals[f"housing_{key}"][1])
        assert visuals[f"post_{key}"][0] == pytest.approx(x, abs=1e-6)
        assert abs(visuals[f"post_{key}"][1]) > abs(y)


def test_no_part_of_a_side_prop_can_enter_the_top_heads_item_mask():
    """The load-bearing safety claim of the model header, as arithmetic.

    The post is the only part low enough to enter the top head's cone at all.
    Where it does, it is FARTHER than the belt, and `_item_mask` admits only
    pixels nearer than `BELT_DEPTH_M - MASK_MARGIN_M`. If a future edit moves the
    props inward or raises the mask margin, this fails instead of the cell
    quietly sorting its own tripod.
    """
    visuals = _visuals(PROPS.read_text(encoding="utf-8"))
    mask_limit_m = BELT_DEPTH_M - MASK_MARGIN_M
    for key in ("neg_y", "pos_y"):
        near_edge = abs(visuals[f"post_{key}"][1]) - POST_HALF_W_M
        # Highest z at which the post is still inside the cone: solve
        # half_view(z) == near_edge for z.
        z_enter = CAMERA_Z_M - near_edge / ((IMG_W / 2.0) / FX)
        assert z_enter < 0.4, f"{key} post enters the cone above the belt (z={z_enter:.3f})"
        assert CAMERA_Z_M - z_enter > mask_limit_m, (
            f"{key} post is nearer than the belt where it enters the frame")

    for key in ("neg_y", "pos_y"):
        for part in ("housing", "lens"):
            y, z = visuals[f"{part}_{key}"][1], visuals[f"{part}_{key}"][2]
            assert abs(y) - 0.08 > _top_half_view_y_m(z), f"{part}_{key} in the top frame"
