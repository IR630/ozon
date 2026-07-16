from src.hard_scene_metrics import Detection, SceneFrame, TruthItem, evaluate_hard_scenes


def test_clean_sequence_has_no_hard_scene_errors():
    frames = (
        SceneFrame((TruthItem("helmet", "B"),), (Detection(1, ("helmet",), "B"),)),
        SceneFrame((TruthItem("helmet", "B"),), (Detection(1, ("helmet",), "B"),)),
    )

    metrics = evaluate_hard_scenes(frames)

    assert metrics.split_events == 0
    assert metrics.merge_events == 0
    assert metrics.phantom_ids == 0
    assert (metrics.category_correct, metrics.category_total) == (2, 2)


def test_split_merge_phantom_and_category_flip_are_counted_separately():
    frames = (
        SceneFrame((TruthItem("helmet", "B"),), (Detection(1, ("helmet",), "B"),)),
        SceneFrame(
            (TruthItem("helmet", "B"),),
            (Detection(1, ("helmet",), "B"), Detection(2, ("helmet",), "B")),
        ),
        SceneFrame(
            (TruthItem("pouf", "C"), TruthItem("pen", "C")),
            (Detection(3, ("pouf", "pen"), "C"), Detection(9, (), "B")),
        ),
        SceneFrame((TruthItem("helmet", "B"),), (Detection(1, ("helmet",), "D"),)),
    )

    metrics = evaluate_hard_scenes(frames)

    assert metrics.split_events == 1
    assert metrics.merge_events == 1
    assert metrics.phantom_ids == 2
    assert (metrics.category_correct, metrics.category_total) == (2, 3)
