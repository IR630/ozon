from types import SimpleNamespace

import measure_validation as validation
import pytest


def _case(**overrides):
    values = {
        "name": "fixture",
        "depth": validation.IMG / "unused.png",
        "rgb": None,
        "truth_dims_mm": (100.0, 50.0, 20.0),
        "truth_k": 0.7,
        "expected_category": "B",
        "expected_detected": True,
    }
    values.update(overrides)
    return validation.ValidationCase(**values)


def test_evaluation_reports_dimension_and_k_error_and_route():
    result = validation.evaluate(
        _case(),
        SimpleNamespace(dims_mm=[102.0, 47.0, 20.0], k=0.75),
    )

    assert result.detected
    assert result.max_dim_error_mm == 3.0
    assert result.k_error == pytest.approx(0.05)
    assert result.category == "B"
    assert result.passed


def test_visible_miss_fails_but_expected_partial_reject_passes():
    assert not validation.evaluate(_case(), None).passed
    partial = _case(
        truth_dims_mm=None,
        truth_k=None,
        expected_category=None,
        expected_detected=False,
    )
    assert validation.evaluate(partial, None).passed


def test_cli_report_is_encodable_on_the_default_russian_windows_console(
    tmp_path, monkeypatch, capsys
):
    depth = tmp_path / "depth.png"
    depth.write_bytes(b"fixture")
    case = _case(depth=depth)
    monkeypatch.setattr(validation, "CASES", (case,))
    monkeypatch.setattr(validation, "load_depth_png", lambda _path: object())
    monkeypatch.setattr(
        validation,
        "measure_item",
        lambda _depth: SimpleNamespace(dims_mm=[100.0, 50.0, 20.0], k=0.7),
    )

    assert validation.main() == 0
    capsys.readouterr().out.encode("cp1251", errors="strict")


def test_every_frozen_real_slice_passes_the_production_baseline():
    pytest.importorskip("cv2")
    from src.perception import load_depth_png, measure_item

    for case in validation.CASES:
        assert case.depth.exists(), case.depth
        if case.rgb is not None:
            assert case.rgb.exists(), case.rgb
        result = validation.evaluate(case, measure_item(load_depth_png(case.depth)))
        assert result.passed, (case.name, result)
