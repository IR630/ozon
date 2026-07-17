# -*- coding: utf-8 -*-
"""Domain constants from the task statement (docs/md/task.md).

Single source of truth: duplicating these numbers anywhere else in the code
is a bug (CLAUDE.md). Units: linear dimensions — millimeters, time — seconds,
speeds — m/s, angles — radians.
"""

# Sorter limits: item fits iff its dims, sorted descending, are strictly
# inside (MIN, MAX) bounds.
MIN_DIMS_MM = (10.0, 10.0, 10.0)
MAX_DIMS_MM = (450.0, 320.0, 320.0)  # sorted descending

# Circle-in-section criterion: K = r_inscribed / R_circumscribed,
# round iff K is STRICTLY greater than the threshold.
ROUND_K_THRESHOLD = 0.8

BELT_SPEED_M_S = 1.0

# Positional diverter is considered parked only while its complete 0.80 x 0.03 m
# collision box remains outside the belt edge at |y|=0.25. With pivot |y|=0.28,
# 0.015 rad leaves about 3 mm geometric clearance (sim/worlds/cell_diverter.sdf).
DIVERTER_PARK_TOL_RAD = 0.015

# Sanity bounds for measured dims (Karpathy principle 6: no physical dim
# is ever 0 or 5 meters in this task; input items are <= 500 mm).
SANE_DIM_MM_MIN = 1.0
SANE_DIM_MM_MAX = 1000.0

# Categories (routing zones).
CATEGORY_B = "B"  # подходит для сортировки
CATEGORY_C = "C"  # не подходит по габаритам
CATEGORY_D = "D"  # доупаковка (круг в сечении)
