"""Run the production baseline on frozen synthetic/real difficult scenes."""
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from measure_validation import CASES as VALIDATION_CASES  # noqa: E402
from src.classification import classify_conservative  # noqa: E402
from src.hard_scene_metrics import (  # noqa: E402
    Detection,
    HardSceneMetrics,
    SceneFrame,
    TruthItem,
    evaluate_hard_scenes,
)
from src.item_tracking import ItemTracker  # noqa: E402
from src.perception import BELT_DEPTH_M, load_depth_png, measure_items  # noqa: E402


def _validation_case(name):
    return next(case for case in VALIDATION_CASES if case.name == name)


REAL_VISIBLE_CASES = (
    _validation_case("Helmet tilted / 3D body OBB"),
    _validation_case("Plate oi1 / round flat slice"),
    _validation_case("Pen diagonal / thin mask at 45 deg"),
)
REAL_PARTIAL_DEPTHS = tuple(
    _validation_case(name).depth
    for name in (
        "Partial box / border reject",
        "Partial helmet / border reject",
        "Partial pen / thin border reject",
    )
)
EXPECTED = HardSceneMetrics(0, 1, 0, 9, 9)


def _depth(*rectangles):
    depth = np.full((480, 640), BELT_DEPTH_M, dtype=float)
    for row0, row1, col0, col1, value in rectangles:
        depth[row0:row1, col0:col1] = value
    return depth


def _frame(depth, truth, tracker):
    measurements = sorted(measure_items(depth), key=lambda item: item.position_m[1])
    observed_ids = tracker.update([item.position_m for item in measurements])
    if len(measurements) == 1 and len(truth) > 1:
        links = (tuple(item.item_id for item in truth),)
    else:
        links = tuple(
            (truth[index].item_id,) if index < len(truth) else ()
            for index in range(len(measurements))
        )
    detections = tuple(
        Detection(
            observed_id,
            truth_ids,
            classify_conservative(measurement.dims_mm, measurement.k),
        )
        for observed_id, truth_ids, measurement in zip(observed_ids, links, measurements)
    )
    return SceneFrame(tuple(truth), detections)


def build_frames():
    frames = []

    # Supported corner overlap: two visible lobes and a deep saddle.
    frames.append(_frame(
        _depth((100, 220, 100, 220, 1.30), (180, 300, 180, 300, 1.25)),
        (TruthItem("corner_left", "B"), TruthItem("corner_right", "B")),
        ItemTracker(),
    ))
    # Known information limit: a full-edge contact has no top-view neck.
    frames.append(_frame(
        _depth((100, 220, 100, 220, 1.30), (100, 220, 220, 340, 1.30)),
        (TruthItem("edge_left", "B"), TruthItem("edge_right", "B")),
        ItemTracker(),
    ))
    # Partial occlusion at different depths remains two body-like lobes.
    frames.append(_frame(
        _depth((100, 230, 100, 230, 1.30), (150, 280, 180, 310, 1.25)),
        (TruthItem("occluded_left", "B"), TruthItem("occluding_right", "B")),
        ItemTracker(),
    ))

    entry_tracker = ItemTracker()
    frames.extend((
        _frame(_depth((100, 220, 0, 80, 1.30)), (), entry_tracker),
        _frame(
            _depth((100, 220, 70, 190, 1.30)),
            (TruthItem("entering_box", "B"),),
            entry_tracker,
        ),
        _frame(
            _depth((90, 210, 90, 210, 1.30)),
            (TruthItem("entering_box", "B"),),
            entry_tracker,
        ),
        _frame(_depth((80, 200, 560, 640, 1.30)), (), entry_tracker),
    ))

    for index, case in enumerate(REAL_VISIBLE_CASES):
        frames.append(_frame(
            load_depth_png(case.depth),
            (TruthItem(f"real_visible_{index}", case.expected_category),),
            ItemTracker(),
        ))
    for depth_path in REAL_PARTIAL_DEPTHS:
        frames.append(_frame(load_depth_png(depth_path), (), ItemTracker()))
    return tuple(frames)


def main():
    metrics = evaluate_hard_scenes(build_frames())
    print(
        f"split={metrics.split_events} merge={metrics.merge_events} "
        f"phantom_id={metrics.phantom_ids} "
        f"category={metrics.category_correct}/{metrics.category_total}"
    )
    return 0 if metrics == EXPECTED else 1


if __name__ == "__main__":
    raise SystemExit(main())
