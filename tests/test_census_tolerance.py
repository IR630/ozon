# -*- coding: utf-8 -*-
"""The census-log tolerance reader: parsing, the one-line-per-cell rule, scoring."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from census_tolerance import cell_measurements, census_cells, score_run  # noqa: E402

HELMET_TRUTH = (352.0, 298.0, 282.0)

_LINE_2CAM = ("[python3-2] [INFO] [1784706544.155484568] [perception]: "
              "item 1: 347x299x278 mm K=0.70 at (1.87, 0.08) heads=2")
_LINE_1CAM = ("[python3-2] [INFO] [1784706544.155484568] [perception]: "
              "item 1: 349x299x279 mm K=0.71 at (1.87, 0.08)")


def _write(tmp_path, name, lines):
    path = tmp_path / name
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def test_parses_dims_k_and_heads(tmp_path):
    log = _write(tmp_path, "matrix_helmet_0.log", ["starting cell", _LINE_2CAM, "done"])
    assert cell_measurements(log) == [([347.0, 299.0, 278.0], 0.70, 2)]


def test_single_camera_line_without_heads_counts_as_one_head(tmp_path):
    log = _write(tmp_path, "matrix_helmet_0.log", [_LINE_1CAM])
    assert cell_measurements(log)[0][2] == 1


def test_log_without_measurement_yields_no_rows(tmp_path):
    log = _write(tmp_path, "matrix_pen_1.log",
                 ["ABORT: belt never reached full speed", "no measurement in log"])
    assert cell_measurements(log) == []


def test_summary_log_is_not_a_cell(tmp_path):
    _write(tmp_path, "matrix_helmet_0.log", [_LINE_2CAM])
    _write(tmp_path, "summary.log", [_LINE_2CAM])
    assert [(slug, oi) for slug, oi, _ in census_cells(tmp_path)] == [("helmet", 0)]


def test_cell_is_scored_by_its_last_measurement(tmp_path):
    # first frame far outside tolerance, last frame inside: the cell counts as in.
    bad = _LINE_2CAM.replace("347x299x278", "500x299x278")
    _write(tmp_path, "matrix_helmet_0.log", [bad, _LINE_2CAM])
    r = score_run(tmp_path, {"helmet": HELMET_TRUTH})
    assert (r["per_cell_ok"], r["measured"]) == (1, 1)
    # ...while the per-measurement count still shows the bad frame.
    assert (r["per_meas_ok"], r["per_meas_total"]) == (1, 2)
    assert r["multi"] == 1


def test_out_of_tolerance_cell_is_reported_with_its_error(tmp_path):
    _write(tmp_path, "matrix_helmet_0.log",
           [_LINE_2CAM.replace("347x299x278", "500x299x278")])
    r = score_run(tmp_path, {"helmet": HELMET_TRUTH})
    assert r["per_cell_ok"] == 0
    (slug, oi, _dims, side_err, _vol_err), = r["misses"]
    assert (slug, oi) == ("helmet", 0)
    assert side_err == 148.0


def test_cell_without_measurement_stays_in_the_denominator_as_unmeasured(tmp_path):
    _write(tmp_path, "matrix_helmet_0.log", [_LINE_2CAM])
    _write(tmp_path, "matrix_helmet_1.log", ["ABORT: controller did not come up"])
    r = score_run(tmp_path, {"helmet": HELMET_TRUTH})
    assert (r["cells"], r["measured"]) == (2, 1)
    assert r["unmeasured"] == [("helmet", 1)]


def test_heads_are_reported_so_a_top_only_run_cannot_pose_as_multi_head(tmp_path):
    _write(tmp_path, "matrix_helmet_0.log", [_LINE_1CAM])
    _write(tmp_path, "matrix_helmet_1.log", [_LINE_2CAM])
    assert score_run(tmp_path, {"helmet": HELMET_TRUTH})["heads"] == [1, 2]
