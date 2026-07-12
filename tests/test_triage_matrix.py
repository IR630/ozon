# -*- coding: utf-8 -*-
"""Root-cause triage of census cells (scripts/triage_matrix.py).

The census verdict alone (PASS/FAIL) does not say WHERE a run broke, and the
day-6 mechanism decision is read against exactly that split: a misroute is our
rule/perception, a feed jam or an overshoot is the mechanism. These lock the
rules on synthetic episode logs shaped like the real ones (scripts/run_skeleton.sh).
"""
import triage_matrix
from triage_matrix import census_in_flight, diagnose, parse_cell

PERCEPTION = "[python3-2] [INFO] [178.0] [perception]: item 1: {dims} mm K={k} at (1.76, 0.00)"
CLASSIFIER = "[python3-3] [INFO] [178.0] [classifier]: item 1: {cat} (k={k}, conf=0.99, n=9)"
FIRED = "[python3-4] [INFO] [178.0] [controller]: item 1: pusher_{side} FIRED at t=23.2s"


def write_cell(tmp_path, slug, oi, lines):
    path = tmp_path / f"matrix_{slug}_{oi}.log"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return str(path)


def cause_of(tmp_path, slug, oi, lines):
    return diagnose(parse_cell(write_cell(tmp_path, slug, oi, lines))).cause


def verdict(slug, zone, v, x, y, z):
    return f"{slug} -> {zone}: {v} (pose x={x} y={y} z={z}, cycle 25.8s from launch)"


def test_routed_item_is_ok(tmp_path):
    cell = diagnose(
        parse_cell(
            write_cell(
                tmp_path,
                "bottle",
                0,
                [
                    verdict("bottle", "D", "PASS", 3.74, -0.58, 0.058),
                    PERCEPTION.format(dims="298x102x92", k="1.00"),
                    CLASSIFIER.format(cat="D", k="1.000"),
                    FIRED.format(side="d"),
                ],
            )
        )
    )
    assert cell.cause == "ok"
    assert cell.expected == "D"  # read from the log, not duplicated in Python
    assert cell.dims_mm == (298, 102, 92)
    assert cell.cycle_s == 25.8


def test_item_that_never_reached_the_camera_is_a_feed_jam(tmp_path):
    # pouf oi1 in the seed-0 census: final pose still at the spawn end (x=-1.26).
    assert (
        cause_of(tmp_path, "pouf", 1, [verdict("pouf", "C", "FAIL", -1.26, 0.0, 0.5)])
        == "feed_jam"
    )


def test_item_past_the_camera_with_no_measurement_is_a_perception_miss(tmp_path):
    # pen 0/3 before the _MIN_ITEM_PX fix: it rode the whole belt, unseen.
    assert (
        cause_of(tmp_path, "pen", 0, [verdict("pen", "C", "FAIL", 3.90, 0.0, 0.45)])
        == "no_detect"
    )


def test_wrong_category_is_a_misroute(tmp_path):
    # helmet oi0: aggregate K=0.82 crossed the 0.8 threshold -> D instead of B.
    cell = diagnose(
        parse_cell(
            write_cell(
                tmp_path,
                "helmet",
                0,
                [
                    verdict("helmet", "B", "FAIL", 3.6, -0.9, 0.05),
                    PERCEPTION.format(dims="286x281x155", k="0.82"),
                    CLASSIFIER.format(cat="D", k="0.820"),
                    FIRED.format(side="d"),
                ],
            )
        )
    )
    assert cell.cause == "misroute"
    assert "expected B" in cell.detail


def test_wrong_controller_route_beats_a_truncated_perception_line(tmp_path):
    # Real Bag oi1: ROS stdout lost the perception line at shutdown, but the
    # controller proves that a D decision existed.  This is a misroute, not a
    # no-detect failure.
    cell = diagnose(
        parse_cell(
            write_cell(
                tmp_path,
                "bag",
                1,
                [
                    verdict("bag", "B", "FAIL", 3.63, -1.38, 0.08),
                    "[python3-4] [INFO] [178.0] [controller]: item 1: D — firing pusher_d in 0.49s",
                    FIRED.format(side="d"),
                ],
            )
        )
    )

    assert cell.dims_mm is None
    assert cell.category == "D"
    assert cell.cause == "misroute"


def test_correct_category_but_item_stayed_on_the_belt_is_a_mechanism_miss(tmp_path):
    # Classified C correctly, the mechanism fired, yet the item rode past at belt
    # height — the blade/paddle never took it (timing), an EXECUTION failure.
    assert (
        cause_of(
            tmp_path,
            "pouf",
            2,
            [
                verdict("pouf", "C", "FAIL", 4.75, 0.0, 0.50),
                PERCEPTION.format(dims="489x421x265", k="0.61"),
                CLASSIFIER.format(cat="C", k="0.610"),
                FIRED.format(side="c"),
            ],
        )
        == "mech_miss"
    )


def test_correct_category_off_the_belt_but_outside_the_zone_is_an_overshoot(tmp_path):
    assert (
        cause_of(
            tmp_path,
            "box_400x400x300",
            1,
            [
                verdict("box_400x400x300", "C", "FAIL", 3.95, 1.10, 0.02),
                PERCEPTION.format(dims="416x400x299", k="0.60"),
                CLASSIFIER.format(cat="C", k="0.607"),
                FIRED.format(side="c"),
            ],
        )
        == "mech_overshoot"
    )


def test_correct_category_that_never_fired_is_a_timing_failure(tmp_path):
    assert (
        cause_of(
            tmp_path,
            "detergent",
            1,
            [
                verdict("detergent", "C", "FAIL", 4.2, 0.0, 0.45),
                PERCEPTION.format(dims="300x200x100", k="0.72"),
                CLASSIFIER.format(cat="C", k="0.720"),
            ],
        )
        == "no_fire"
    )


def test_b_item_knocked_off_the_belt_is_a_false_divert(tmp_path):
    # A mechanism must not disturb a passing B item (compare_mechanisms.sh).
    assert (
        cause_of(
            tmp_path,
            "box_300x200x200",
            0,
            [
                verdict("box_300x200x200", "B", "FAIL", 3.0, 1.2, 0.01),
                PERCEPTION.format(dims="296x200x198", k="0.56"),
                CLASSIFIER.format(cat="B", k="0.560"),
                FIRED.format(side="c"),
            ],
        )
        == "false_divert"
    )


def test_route_is_read_from_the_controller_when_the_classifier_line_is_lost(tmp_path):
    # bottle oi=1, seed-0 diverter census: killing the launch truncated node
    # stdout, so the cell has the controller's decision but no classifier line.
    # The item was routed D correctly and landed on the floor 20 mm short of the
    # zone window (y=-0.48 vs the -0.5 edge) — a mechanism miss, not a blind run.
    cell = diagnose(
        parse_cell(
            write_cell(
                tmp_path,
                "bottle",
                1,
                [
                    verdict("bottle", "D", "FAIL", 3.738, -0.479, -0.001),
                    PERCEPTION.format(dims="317x101x93", k="1.00"),
                    "[python3-4] [INFO] [178.0] [controller]: item 1: D — firing pusher_d in 0.46s",
                    FIRED.format(side="d"),
                ],
            )
        )
    )
    assert cell.category == "D"
    assert cell.cause == "mech_overshoot"


def test_a_cell_still_running_is_not_scored_as_a_wedge(tmp_path):
    # Triage must be usable mid-census: main() re-labels the in-flight cell
    # (newest log while Gazebo is alive) before diagnosing it.
    cell = parse_cell(write_cell(tmp_path, "helmet", 2, ["[python3-2] [INFO] booting gazebo"]))
    cell.verdict = "RUNNING"
    assert diagnose(cell).cause == "in_progress"


def test_in_flight_probe_only_matches_a_real_gazebo_command(monkeypatch):
    calls = []

    class Result:
        returncode = 0

    def fake_run(args, **kwargs):
        calls.append((args, kwargs))
        return Result()

    monkeypatch.setattr(triage_matrix.subprocess, "run", fake_run)

    assert census_in_flight()
    assert calls == [
        (["pgrep", "-f", r"^ign gazebo( |$)"], {"capture_output": True})
    ]


def test_killed_episode_without_a_verdict_line_is_a_physics_wedge(tmp_path):
    # CELL_TIMEOUT kills the cell mid-episode: run_skeleton.sh never gets to
    # print its verdict, so the log simply ends (pouf, seed-0 census).
    cell = diagnose(
        parse_cell(
            write_cell(tmp_path, "pouf", 1, ["[python3-2] [INFO] booting", "spawning pouf"])
        )
    )
    assert cell.cause == "physics_wedge"
    assert cell.verdict == "TIMEOUT"
    assert cell.expected is None
