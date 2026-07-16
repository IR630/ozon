"""Metrics for annotated perception sequences with difficult item geometry."""
from collections import defaultdict
from dataclasses import dataclass


@dataclass(frozen=True)
class TruthItem:
    item_id: str
    category: str


@dataclass(frozen=True)
class Detection:
    observed_id: int
    truth_ids: tuple[str, ...]
    category: str


@dataclass(frozen=True)
class SceneFrame:
    truth: tuple[TruthItem, ...]
    detections: tuple[Detection, ...]


@dataclass(frozen=True)
class HardSceneMetrics:
    split_events: int
    merge_events: int
    phantom_ids: int
    category_correct: int
    category_total: int


def evaluate_hard_scenes(frames) -> HardSceneMetrics:
    """Count independent geometry, identity and category failures."""
    split_events = 0
    merge_events = 0
    category_correct = 0
    category_total = 0
    observed_ids_by_truth = defaultdict(set)
    unassociated_ids = set()

    for frame in frames:
        truth = {item.item_id: item.category for item in frame.truth}
        detections_by_truth = defaultdict(list)

        for detection in frame.detections:
            if not detection.truth_ids:
                unassociated_ids.add(detection.observed_id)
            if len(detection.truth_ids) > 1:
                merge_events += 1
            for truth_id in detection.truth_ids:
                if truth_id not in truth:
                    raise ValueError(f"unknown truth item {truth_id!r}")
                detections_by_truth[truth_id].append(detection)
                observed_ids_by_truth[truth_id].add(detection.observed_id)

        for truth_id, detections in detections_by_truth.items():
            if len(detections) > 1:
                split_events += 1
            if all(len(detection.truth_ids) == 1 for detection in detections):
                category_total += 1
                if all(detection.category == truth[truth_id] for detection in detections):
                    category_correct += 1

    fragmented_ids = sum(max(0, len(ids) - 1) for ids in observed_ids_by_truth.values())
    return HardSceneMetrics(
        split_events=split_events,
        merge_events=merge_events,
        phantom_ids=len(unassociated_ids) + fragmented_ids,
        category_correct=category_correct,
        category_total=category_total,
    )
