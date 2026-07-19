# -*- coding: utf-8 -*-
"""Domain constants from the task statement (docs/md/task.md).

Single source of truth: duplicating these numbers anywhere else in the code
is a bug (CLAUDE.md). Units: linear dimensions — millimeters, time — seconds,
speeds — m/s.
"""

# Sorter limits: item fits iff its dims, sorted descending, are strictly
# inside (MIN, MAX) bounds.
MIN_DIMS_MM = (10.0, 10.0, 10.0)
MAX_DIMS_MM = (450.0, 320.0, 320.0)  # sorted descending

# Circle-in-section criterion: K = r_inscribed / R_circumscribed,
# round iff K is STRICTLY greater than the threshold.
ROUND_K_THRESHOLD = 0.8

BELT_SPEED_M_S = 1.0

# Measurement accuracy the organizers allow, from the expert session
# (docs/md/expert_session_qa.md [08:45], [26:56]): 5 mm on a side OR 10 % by
# volume, whichever of the two is the more permissive. Stated by example: an item
# measured 451x321x321 against a true 450x320x320 is explicitly NOT an error.
# This is the yardstick applied to OUR measurement — it does NOT widen the
# classification thresholds above, which stay exactly where the task puts them.
MEASUREMENT_TOL_MM = 5.0
MEASUREMENT_TOL_VOLUME_FRAC = 0.10

# Sanity bounds for measured dims (Karpathy principle 6: no physical dim
# is ever 0 or 5 meters in this task; input items are <= 500 mm).
SANE_DIM_MM_MIN = 1.0
SANE_DIM_MM_MAX = 1000.0

# Categories (routing zones).
CATEGORY_B = "B"  # подходит для сортировки
CATEGORY_C = "C"  # не подходит по габаритам
CATEGORY_D = "D"  # доупаковка (круг в сечении)
