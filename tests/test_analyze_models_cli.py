# -*- coding: utf-8 -*-
"""The "expert loads their own STL" entry point (docs/md/expert_session_qa.md).

Organizers require a path where an arbitrary test model goes in and a category
comes out `[35:29]`. The one property worth locking: an ad-hoc run must never
overwrite docs/md/models.md — that table is the project's reference and is
regenerated only by the deliberate no-args run.
"""
from pathlib import Path

import pytest

from scripts.analyze_models import main

ROOT = Path(__file__).resolve().parents[1]
CYLINDER = ROOT / "docs" / "Stl" / "Цилиндр.stl"


class TestAdHocRun:
    def test_classifies_a_single_model(self, capsys):
        assert main([str(CYLINDER)]) == 0
        out = capsys.readouterr().out
        assert "Цилиндр" in out and "K=" in out

    def test_does_not_touch_the_reference_table(self, capsys):
        models_md = ROOT / "docs" / "md" / "models.md"
        before = models_md.read_bytes()
        main([str(CYLINDER)])
        assert models_md.read_bytes() == before, "ad-hoc run overwrote models.md"

    def test_rejects_a_missing_file(self):
        with pytest.raises(SystemExit) as e:
            main(["definitely_not_here.stl"])
        assert e.value.code == 2
