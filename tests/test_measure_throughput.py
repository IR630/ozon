# -*- coding: utf-8 -*-
"""Parser + statistics for scripts/measure_throughput.py, on a synthetic run.

The three clocks and namespaces are the tricky part (see the module docstring), so
this pins them with hand-written logs whose every stamp is known: camera->decision
and decision->command come out of skeleton.log, the takt out of stream.log, and the
two are NOT joined across their different item-id namespaces.
"""
import measure_throughput as mt
from src.constants import BELT_SPEED_M_S

# item 1 fires (full chain); item 2 has NO classifier line (truncated) -> the
# controller decision line is the fallback; item 3 is B and never fires. Item 1's
# perception line appears twice to prove the FIRST detection wins.
SKELETON = """\
[python3-2] [INFO] [1000.000000000] [perception]: item 1: 300x100x90 mm K=1.00 at (1.5, 0.1)
[python3-2] [INFO] [1000.200000000] [perception]: item 1: 301x101x90 mm K=1.00 at (1.7, 0.1)
[python3-3] [INFO] [1000.500000000] [classifier]: item 1: D (k=1.000, conf=0.98, n=3)
[python3-4] [INFO] [1000.600000000] [controller]: item 1: D — firing pusher_d in 0.40s
[python3-4] [INFO] [1001.000000000] [controller]: item 1: pusher_d FIRED at t=24.01s
[python3-2] [INFO] [1002.000000000] [perception]: item 2: 200x150x140 mm K=0.70 at (1.5, -0.1)
[python3-4] [INFO] [1002.800000000] [controller]: item 2: C — firing pusher_c in 0.40s
[python3-4] [INFO] [1003.400000000] [controller]: item 2: pusher_c FIRED at t=26.00s
[python3-2] [INFO] [1004.000000000] [perception]: item 3: 250x200x180 mm K=0.55 at (1.5, 0.0)
[python3-3] [INFO] [1004.300000000] [classifier]: item 3: B (k=0.550, conf=0.90, n=2)
[python3-4] [INFO] [1004.400000000] [controller]: item 3: B — rides to belt end
"""

# stream.log uses the SPAWN index (item0..), a different namespace from skeleton's
# tracker ids. A FAILed item has no arrival time and must be ignored.
STREAM = """\
=== stream result ===
item0 box_400x400x300 -> C: PASS at t=3.0s (x=3.0 y=1.0 z=0.0)
item1 pen -> C: PASS at t=4.3s (x=3.1 y=1.1 z=0.0)
item2 bottle -> D: PASS at t=10.0s (x=3.5 y=-0.8 z=0.0)
item3 helmet -> B: FAIL (no pose)
routed 3/3
"""


def _run(tmp_path):
    (tmp_path / "skeleton.log").write_text(SKELETON, encoding="utf-8")
    (tmp_path / "stream.log").write_text(STREAM, encoding="utf-8")
    return tmp_path


def test_skeleton_stages_and_first_detection(tmp_path):
    items = mt.parse_skeleton(_run(tmp_path) / "skeleton.log")
    assert set(items) == {1, 2, 3}
    # First perception stamp wins, not the later one.
    assert items[1].detect == 1000.0
    assert items[1].decide == 1000.5      # classifier
    assert items[1].fire == 1001.0
    # No classifier line -> controller decision line is the decision.
    assert items[2].decide_cls is None
    assert items[2].decide == 1002.8      # falls back to controller "C — firing"
    assert items[2].fire == 1003.4
    # B item rides: a decision, but never a fire.
    assert items[3].decide == 1004.3
    assert items[3].fire is None


def test_latency_segments(tmp_path):
    items = mt.parse_skeleton(_run(tmp_path) / "skeleton.log")
    # Rounded: subtracting ~1e9 epoch stamps loses the last digits (real behaviour).
    cam = sorted(round(s.decide - s.detect, 3) for s in items.values())
    fire = sorted(round(s.fire - s.decide, 3) for s in items.values() if s.fire is not None)
    assert cam == [0.3, 0.5, 0.8]            # items 3, 1, 2
    assert fire == [0.5, 0.6]                 # items 1, 2 (item 3 never fired)


def test_stream_arrivals_pass_only_and_sorted(tmp_path):
    arr = mt.parse_stream(_run(tmp_path) / "stream.log")
    assert [a.name for a in arr] == ["item0", "item1", "item2"]  # FAIL dropped
    assert [a.zone for a in arr] == ["C", "C", "D"]
    assert [a.t for a in arr] == [3.0, 4.3, 10.0]


def test_takt_gaps_and_computed_floor(tmp_path):
    arr = mt.parse_stream(_run(tmp_path) / "stream.log")
    gaps = mt.takt_gaps(arr)
    assert [round(g.gap_s, 1) for g in gaps] == [1.3, 5.7]
    # C->C rides nose to tail: no floor. C->D needs the blade's hold+retract air.
    assert gaps[0].expected_s == 0.0
    change_floor = mt.stream_plan.min_gap_between_zones_m("C", "D") / BELT_SPEED_M_S
    assert gaps[1].expected_s == change_floor
    assert change_floor > 0


def test_percentile():
    assert mt.percentile([], 95) is None
    assert mt.percentile([7], 95) == 7
    assert mt.percentile([1, 2, 3, 4, 5], 50) == 3
    assert mt.percentile([0, 10], 50) == 5
    assert mt.percentile([0, 10], 95) == 9.5


def test_find_runs_direct_and_parent(tmp_path):
    run = tmp_path / "stream_A"
    run.mkdir()
    _run(run)
    # A parent dir resolves to the run(s) beneath it; a run dir resolves to itself.
    assert mt.find_runs([str(tmp_path)]) == [run]
    assert mt.find_runs([str(run)]) == [run]
