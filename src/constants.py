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

# Joint hard stops are 0..0.95 rad, but the occupied E-stop smoke observed the
# loaded blade deflect to about 1.24 rad. Keep that real state valid with a small
# margin while rejecting corrupt finite sensor values before they can become
# actuator targets.
DIVERTER_FEEDBACK_MIN_RAD = -0.10
DIVERTER_FEEDBACK_MAX_RAD = 1.30

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


# --- Three-head camera rig (branch feat/three-cameras) --------------------------
#
# Poses of ALL heads live here and nowhere else, so the world file, the node and
# the probes cannot drift apart. Each entry is (position_m, look_at_m) in world
# coordinates; the belt runs along +x and the sorter's belt edges are at |y|=0.25.
#
# THE SIDE HEADS' HEIGHT IS A CONSTRAINT, NOT A PREFERENCE. A body inside the top
# head's cone lands in its mask and _find_item returns None on every frame — a
# silent, total failure. The cone is 869 mm wide at BELT level and only 666 mm at
# the side heads' mounting height, so the same |y| = 0.90 m that clears by 189 mm
# up here would sit 14 mm INSIDE the frame down at the belt (measured with a
# 90 mm D435-class housing, docs/report/img/three_camera_layout_yz.png). Lower
# these heads and the rig breaks silently. test_three_camera_layout.py guards it.
CAMERA_TOP_POSE_M = ((1.5, 0.0, 1.9), (1.5, 0.0, 0.4))
CAMERA_SIDE_NEG_Y_POSE_M = ((1.5, -0.90, 0.75), (1.5, 0.0, 0.5))
CAMERA_SIDE_POS_Y_POSE_M = ((1.5, +0.90, 0.75), (1.5, 0.0, 0.5))
CAMERA_RIG_POSES_M = (CAMERA_TOP_POSE_M,
                      CAMERA_SIDE_NEG_Y_POSE_M,
                      CAMERA_SIDE_POS_Y_POSE_M)

# Belt travel between two heads' frames is the sync penalty the brief calls out:
# untriggered 15 Hz heads can be a full frame apart, and at 1 m/s that is 66.7 mm
# against a 5 mm accuracy budget. The node compensates by shifting each head's
# cloud along +x by (t_reference - t_head) * BELT_SPEED_M_S, so this number is the
# size of the error being cancelled, not an allowance to live with.
CAMERA_FRAME_PERIOD_S = 1.0 / 15.0
