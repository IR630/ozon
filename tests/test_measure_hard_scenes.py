import measure_hard_scenes as hard_scenes

from src.hard_scene_metrics import evaluate_hard_scenes


def test_hard_scene_dataset_has_measured_baseline():
    metrics = evaluate_hard_scenes(hard_scenes.build_frames())

    assert metrics.split_events == 0
    assert metrics.merge_events == 1
    assert metrics.phantom_ids == 0
    assert (metrics.category_correct, metrics.category_total) == (9, 9)


def test_hard_scene_dataset_contains_synthetic_and_real_inputs():
    assert len(hard_scenes.REAL_VISIBLE_CASES) == 3
    assert len(hard_scenes.REAL_PARTIAL_DEPTHS) == 3
