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

PLAN = """\
=== stream plan (world=sim/worlds/cell_diverter.sdf seed=0 orient=0) ===
0 box_400x400x300 C -1.500 0.00
1 pen C -1.500 1.00
2 bottle D -1.500 4.50
3 helmet B -1.500 7.60
"""


def _run(tmp_path):
    (tmp_path / "skeleton.log").write_text(SKELETON, encoding="utf-8")
    (tmp_path / "stream.log").write_text(STREAM, encoding="utf-8")
    (tmp_path / "plan.log").write_text(PLAN, encoding="utf-8")
    return tmp_path


def test_skeleton_stages_first_line_wins(tmp_path):
    items = mt.parse_skeleton(_run(tmp_path) / "skeleton.log")
    assert set(items) == {1, 2, 3}
    # Every stage is the EARLIEST line of its kind (first detection, first commit).
    assert items[1].detect == 1000.0        # first perception, not the 1000.2 line
    assert items[1].classify == 1000.5      # first ItemClassification publish
    assert items[1].commit == 1000.6        # controller route commit ("D — firing")
    assert items[1].fire == 1001.0
    # No classifier line at all (truncated) -> classify is None, but commit/fire hold.
    assert items[2].classify is None
    assert items[2].commit == 1002.8        # "C — firing"
    assert items[2].fire == 1003.4
    # B item rides: a commit, but never a fire.
    assert items[3].commit == 1004.4        # "B — rides to belt end"
    assert items[3].fire is None


def test_reliable_latency_segments(tmp_path):
    """decision->command and camera->command; camera->decision is NOT computed
    (below the cross-node jitter floor — the reframe that killed the silent drop)."""
    items = mt.parse_skeleton(_run(tmp_path) / "skeleton.log")
    # Rounded: subtracting ~1e9 epoch stamps loses the last digits (real behaviour).
    dec_cmd = sorted(round(s.fire - s.commit, 3)
                     for s in items.values() if s.fire is not None)
    cam_cmd = sorted(round(s.fire - s.detect, 3)
                     for s in items.values() if s.fire is not None)
    assert dec_cmd == [0.4, 0.6]             # items 1, 2 (commit->FIRED)
    assert cam_cmd == [1.0, 1.4]             # items 1, 2 (detect->FIRED); item 3 never fired


def test_jitter_inverted_stamps_not_dropped(tmp_path):
    """Real data: the controller can log its commit BEFORE perception logs the
    frame (cross-node jitter). The reframed segments (fire-anchored) must still
    yield a value — the first cut's `>= detect` guard silently dropped these."""
    log = tmp_path / "skeleton.log"
    log.write_text(
        # commit @ .3 and classify @ .4 both PRECEDE the first perception @ .5
        "[python3-4] [INFO] [2000.300000000] [controller]: item 9: D — firing pusher_d in 0.9s\n"
        "[python3-3] [INFO] [2000.400000000] [classifier]: item 9: D (k=1.0, conf=0.2, n=1)\n"
        "[python3-2] [INFO] [2000.500000000] [perception]: item 9: 300x100x90 mm K=1.0 at (1.2, 0.2)\n"
        "[python3-4] [INFO] [2001.200000000] [controller]: item 9: pusher_d FIRED at t=24.0s\n",
        encoding="utf-8",
    )
    s = mt.parse_skeleton(log)[9]
    assert round(s.fire - s.commit, 3) == 0.9    # decision->command, reliable
    assert round(s.fire - s.detect, 3) == 0.7    # camera->command, still produced


def test_stream_arrivals_pass_only_and_sorted(tmp_path):
    arr = mt.parse_stream(_run(tmp_path) / "stream.log")
    assert [a.name for a in arr] == ["item0", "item1", "item2"]  # FAIL dropped
    assert [a.zone for a in arr] == ["C", "C", "D"]
    assert [a.t for a in arr] == [3.0, 4.3, 10.0]


def test_stream_results_keep_failures_for_reliability(tmp_path):
    results = mt.parse_stream_results(_run(tmp_path) / "stream.log")
    assert [r.name for r in results] == ["item0", "item1", "item2", "item3"]
    assert [r.passed for r in results] == [True, True, True, False]
    assert results[-1].slug == "helmet"
    assert results[-1].zone == "B"
    assert results[-1].t is None


def test_reliability_counts_episodes_items_and_each_route(tmp_path):
    first = mt.parse_stream_results(_run(tmp_path) / "stream.log")
    second_log = tmp_path / "second.log"
    second_log.write_text(
        "item0 box_400x400x300 -> C: PASS at t=3.2s (x=3 y=1 z=0)\n"
        "item1 helmet -> B: PASS at t=8.0s (x=5 y=0 z=0.4)\n",
        encoding="utf-8",
    )
    second = mt.parse_stream_results(second_log)

    summary = mt.summarize_reliability([first, [], second])
    assert (summary.episodes, summary.all_pass_episodes) == (2, 1)
    assert (summary.items, summary.passed_items) == (6, 5)
    assert summary.by_route[("box_400x400x300", "C")] == (2, 2)
    assert summary.by_route[("helmet", "B")] == (1, 2)


def test_malformed_pass_without_time_is_not_claimed_as_a_result(tmp_path):
    log = tmp_path / "stream.log"
    log.write_text(
        "item0 pen -> C: PASS (x=3 y=1 z=0)\n"
        "item1 bottle -> D: FAIL (x=4 y=0 z=0.4)\n",
        encoding="utf-8",
    )
    results = mt.parse_stream_results(log)
    assert [(r.slug, r.passed) for r in results] == [("bottle", False)]


def test_takt_gaps_and_computed_floor(tmp_path):
    arr = mt.parse_stream(_run(tmp_path) / "stream.log")
    gaps = mt.takt_gaps(arr)
    assert [round(g.gap_s, 1) for g in gaps] == [1.3, 5.7]
    # C->C rides nose to tail: no floor. C->D needs the blade's hold+retract air.
    assert gaps[0].feed_floor_s == 0.0
    change_floor = mt.stream_plan.min_gap_between_zones_m("C", "D") / BELT_SPEED_M_S
    assert gaps[1].feed_floor_s == change_floor
    assert change_floor > 0


def test_saved_plan_preserves_feed_gaps_separately_from_arrival_takt(tmp_path):
    plan = mt.parse_plan(_run(tmp_path) / "plan.log")
    assert [(item.slug, item.zone, item.feed_s) for item in plan] == [
        ("box_400x400x300", "C", 0.0),
        ("pen", "C", 1.0),
        ("bottle", "D", 4.5),
        ("helmet", "B", 7.6),
    ]
    # D->B is fed exactly at the 3.1 s mechanism floor. Its ARRIVAL difference
    # may be smaller because D and B have different paths; that is not a violation.
    assert round(plan[3].feed_s - plan[2].feed_s, 6) == mt.takt_floor_s("D", "B")


def test_takt_floor_converts_distance_to_time_at_the_selected_belt_speed():
    # At 2 m/s the required distance doubles, while hold+retract time stays fixed.
    floor_m = mt.stream_plan.min_gap_between_zones_m(
        "C", "D", belt_speed_m_s=2.0)
    assert mt.takt_floor_s("C", "D", belt_speed_m_s=2.0) == floor_m / 2.0
    assert mt.takt_floor_s("C", "C", belt_speed_m_s=2.0) == 0.0


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
