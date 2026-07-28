# -*- coding: utf-8 -*-
"""Offline regression tests for edge inputs found by the robustness audit."""
from types import SimpleNamespace

from src.perception import _silhouette_solidity


def test_silhouette_solidity_survives_degenerate_hull():
    assert _silhouette_solidity([], SimpleNamespace(volume=0.0)) == 0.0
