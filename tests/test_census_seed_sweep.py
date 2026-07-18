import census_seed_sweep as sweep
from census_seed_sweep import Record, summarize


def test_seed_label_parsed_from_run_matrix_dirname():
    assert sweep._seed_of("runs/matrix_20260718_090549_seed3") == "3"
    assert sweep._seed_of("runs/matrix_x_seed0/") == "0"


def test_all_pass_across_seeds_reports_no_seed_sensitivity():
    records = [
        Record(seed, "bottle", oi, "ok", "D")
        for seed in ("0", "1", "2")
        for oi in (0, 1, 2)
    ]
    s = summarize(records)

    assert (s.routed, s.total) == (9, 9)
    assert s.nondeterministic == []
    assert s.class_failures == 0 and s.exec_failures == 0


def test_cell_that_fails_on_one_seed_only_is_flagged_nondeterministic():
    records = [
        Record("0", "pouf", 1, "ok", "C"),
        Record("1", "pouf", 1, "mech_overshoot", "C"),  # execution fail on seed 1
        Record("2", "pouf", 1, "ok", "C"),
        Record("0", "helmet", 0, "ok", "B"),
        Record("1", "helmet", 0, "ok", "B"),
    ]
    s = summarize(records)

    assert (s.routed, s.total) == (4, 5)
    assert len(s.nondeterministic) == 1
    slug, orient, seed_cause = s.nondeterministic[0]
    assert (slug, orient) == ("pouf", 1)
    assert seed_cause == {"0": "ok", "1": "mech_overshoot", "2": "ok"}


def test_failures_split_classification_vs_execution_like_triage():
    records = [
        Record("0", "plate", 0, "misroute", "D"),        # classification
        Record("0", "box_400x400x300", 0, "feed_jam", "C"),  # execution
        Record("0", "pen", 0, "no_detect", "C"),         # classification
    ]
    s = summarize(records)

    assert s.class_failures == 2
    assert s.exec_failures == 1
    assert s.routed == 0 and s.total == 3
