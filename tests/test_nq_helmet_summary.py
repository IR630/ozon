# -*- coding: utf-8 -*-
"""The K-distribution summary must parse the REAL node-log format strings.

These fixtures reproduce the exact lines emitted by src/perception_node.py
(per-frame `... mm K=0.XXXX ...`) and src/classifier_node.py (aggregate
`item N: <B|C|D> (k=0.XXXXXX, ...)`). If either format changes, this test breaks
before a night run is silently mis-parsed into a wrong diagnosis.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from nq_helmet_summary import parse_node_log, summarize  # noqa: E402

# Verbatim format: src/perception_node.py line 175-178 (K to 4 decimals) and
# src/classifier_node.py line 47-49 (k to 6 decimals). A D verdict cell.
NODE_LOG_MISS = """\
[bringup] soft-start done
item 1: 324x298x284 mm K=0.7996 at (0.01, 0.02) heads=1
item 1: B (k=0.799600, conf=0.40, n=1)
item 1: 324x298x284 mm K=0.8000 at (0.01, 0.02) heads=1
item 1: B (k=0.799800, conf=0.55, n=2)
item 1: 325x298x284 mm K=0.8003 at (0.01, 0.02) heads=1
item 1: D (k=0.800000, conf=0.83, n=3)
"""

NODE_LOG_PASS = """\
item 1: 322x298x284 mm K=0.7990 at (0.01, 0.02) heads=1
item 1: B (k=0.799000, conf=0.83, n=1)
item 1: 323x298x284 mm K=0.7995 at (0.01, 0.02) heads=1
item 1: B (k=0.799200, conf=0.90, n=2)
"""


def test_parse_extracts_per_frame_ks_and_final_verdict(tmp_path):
    p = tmp_path / "helmet_seed1_oi1_rep1.node.log"
    p.write_text(NODE_LOG_MISS, encoding="utf-8")
    rep = parse_node_log(str(p))
    assert rep.frame_ks == [0.7996, 0.8000, 0.8003]
    # Final verdict is the LAST running-median line, not the first.
    assert rep.verdict_zone == "D"
    assert rep.verdict_k == 0.8


def test_pass_cell_reads_as_B(tmp_path):
    p = tmp_path / "helmet_seed1_oi1_rep2.node.log"
    p.write_text(NODE_LOG_PASS, encoding="utf-8")
    rep = parse_node_log(str(p))
    assert rep.frame_ks == [0.7990, 0.7995]
    assert rep.verdict_zone == "B"


def test_summarize_rolls_up_by_cell(tmp_path, capsys):
    (tmp_path / "helmet_seed1_oi1_rep1.node.log").write_text(NODE_LOG_MISS, encoding="utf-8")
    (tmp_path / "helmet_seed1_oi1_rep2.node.log").write_text(NODE_LOG_PASS, encoding="utf-8")
    rc = summarize(str(tmp_path))
    assert rc == 0
    out = capsys.readouterr().out
    # Two repeats of one cell, one of them D.
    assert "helmet_seed1_oi1: 2 repeats" in out
    assert "verdict D in 1/2" in out


def test_missing_dir_is_error(tmp_path):
    assert summarize(str(tmp_path / "nope")) == 1
