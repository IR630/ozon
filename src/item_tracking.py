"""Small deterministic multi-item tracker for a one-direction conveyor."""
from dataclasses import dataclass

import numpy as np

MAX_MATCH_DISTANCE_M = 0.25
MAX_MISSED_FRAMES = 2


@dataclass
class _Track:
    position: np.ndarray
    missed: int = 0


class ItemTracker:
    """Assign stable integer IDs to per-frame world positions.

    Products travel in the same direction and do not overtake, so global visual
    re-identification is unnecessary for the first multi-item contour. Greedy
    nearest-neighbour matching is deterministic, order-independent and explicit
    about its distance/missed-frame gates.
    """

    def __init__(self, max_distance_m=MAX_MATCH_DISTANCE_M,
                 max_missed_frames=MAX_MISSED_FRAMES):
        self.max_distance_m = float(max_distance_m)
        self.max_missed_frames = int(max_missed_frames)
        self._tracks = {}
        self._next_id = 1

    def update(self, positions):
        """Return track IDs aligned with positions from the current frame."""
        points = [np.asarray(position, dtype=float) for position in positions]
        for track in self._tracks.values():
            track.missed += 1

        # Build every admissible edge, then take shortest disjoint pairs. Sorting
        # by IDs/indices after distance makes ties deterministic.
        edges = []
        for item_id, track in self._tracks.items():
            for detection_index, point in enumerate(points):
                distance = float(np.linalg.norm(point[:2] - track.position[:2]))
                if distance <= self.max_distance_m:
                    edges.append((distance, item_id, detection_index))
        edges.sort()

        result = [None] * len(points)
        matched_tracks = set()
        for _, item_id, detection_index in edges:
            if item_id in matched_tracks or result[detection_index] is not None:
                continue
            result[detection_index] = item_id
            matched_tracks.add(item_id)

        for detection_index, point in enumerate(points):
            item_id = result[detection_index]
            if item_id is None:
                item_id = self._next_id
                self._next_id += 1
                result[detection_index] = item_id
                self._tracks[item_id] = _Track(point)
            else:
                self._tracks[item_id].position = point
                self._tracks[item_id].missed = 0

        for item_id in list(self._tracks):
            if self._tracks[item_id].missed > self.max_missed_frames:
                del self._tracks[item_id]
        return result

