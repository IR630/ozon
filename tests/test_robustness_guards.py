# -*- coding: utf-8 -*-
"""Defensive guards from the 2026-07-24 robustness audit (offline-testable half).

A 15 Hz perception node must not crash on a degenerate hull: the old
`assert hull_area > 0` (which also vanishes under `python -O`, leaving a raw
ZeroDivisionError) is replaced by returning 0.0. The classifier-node half of the
audit (drop an out-of-contract measurement instead of dying) needs the ROS graph
and lives in tests/test_classifier_node_robustness.py.
"""
from types import SimpleNamespace

from src.perception import _silhouette_solidity


def test_silhouette_solidity_survives_degenerate_hull():
    # A zero-area hull previously tripped the assert / divided by zero; now it
    # reads "not solid" (0.0) and the frame keeps flowing.
    assert _silhouette_solidity([], SimpleNamespace(volume=0.0)) == 0.0
