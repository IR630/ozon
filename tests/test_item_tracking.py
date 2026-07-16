"""Contract tests for stable IDs with several conveyor items."""
from src.item_tracking import ItemTracker


def test_two_items_keep_ids_when_detection_order_changes():
    tracker = ItemTracker()
    assert tracker.update([(1.0, 0.0, 0.4), (1.4, 0.0, 0.4)]) == [1, 2]

    # Both advanced along +X; detector order is reversed.
    assert tracker.update([(1.47, 0.0, 0.4), (1.07, 0.0, 0.4)]) == [2, 1]


def test_one_missed_frame_does_not_change_id():
    tracker = ItemTracker()
    assert tracker.update([(1.0, 0.0, 0.4)]) == [1]
    assert tracker.update([]) == []
    assert tracker.update([(1.12, 0.0, 0.4)]) == [1]


def test_tumbling_item_keeps_id_after_three_frame_occlusion():
    """A moving Helmet must not become a phantom item after a short dropout."""
    tracker = ItemTracker()
    assert tracker.update([(1.00, 0.0, 0.4)]) == [1]
    assert tracker.update([(1.07, 0.0, 0.4)]) == [1]

    tracker.update([])
    tracker.update([])
    tracker.update([])

    assert tracker.update([(1.35, 0.0, 0.4)]) == [1]


def test_tumbling_item_does_not_steal_neighbour_id_after_occlusion():
    tracker = ItemTracker()
    assert tracker.update([(0.50, 0.0, 0.4), (1.00, 0.0, 0.4)]) == [1, 2]
    assert tracker.update([(0.57, 0.0, 0.4), (1.07, 0.0, 0.4)]) == [1, 2]

    assert tracker.update([(0.64, 0.0, 0.4)]) == [1]
    assert tracker.update([(0.71, 0.0, 0.4)]) == [1]
    assert tracker.update([(0.78, 0.0, 0.4)]) == [1]

    assert tracker.update([(0.85, 0.0, 0.4), (1.35, 0.0, 0.4)]) == [1, 2]


def test_expired_track_gets_a_new_id():
    tracker = ItemTracker(max_missed_frames=1)
    assert tracker.update([(1.0, 0.0, 0.4)]) == [1]
    tracker.update([])
    tracker.update([])
    assert tracker.update([(1.0, 0.0, 0.4)]) == [2]


def test_far_detection_cannot_steal_existing_id():
    tracker = ItemTracker(max_distance_m=0.2)
    assert tracker.update([(1.0, 0.0, 0.4)]) == [1]
    assert tracker.update([(1.5, 0.0, 0.4)]) == [2]
