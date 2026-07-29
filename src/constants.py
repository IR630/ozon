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

# How far above the belt a side head's point must sit to count as goods.
#
# THIS IS NOT THE TOP HEAD'S 5 mm, AND REUSING THAT NUMBER COST THE BRANCH A
# CENSUS. perception.MASK_MARGIN_M = 5 mm was derived for a straight-down view,
# where the empty belt spreads about 1 mm and a pointing error moves the belt
# SIDEWAYS. A side head grazes the belt plane, so the same pointing error tilts
# that plane about the lens and lifts it: measured on a dumped frame at the
# +-2 mm / 0.2 deg calibration budget, the belt reconstructs 5.00-5.23 mm above
# itself and walks straight through a 5 mm floor. The crop then admits two
# strips of belt edge spanning its whole +-225 mm window, and a 303 mm bottle
# reads 740x505 mm — C instead of D, on every frame (runs/census_3cam_miscal,
# 2 of 8 cells, docs/report/img/side_cloud_miscal.png).
#
# The bound is the translation error plus the rotation error over the longest
# range the crop admits: 2 mm + 0.2 deg x 1.28 m = 6.5 mm for the largest item
# in the catalogue. 8 mm clears that and still sits under the 9 mm pen, which is
# the thinnest item and the one the side heads exist for.
#
# It costs almost nothing because it cuts from the BOTTOM: the side heads'
# contribution to height is a MAXIMUM over the cloud, so trimming low points
# leaves the height untouched, and the x/y shadow is floored by the top head's
# own unoccluded view anyway.
SIDE_BELT_MARGIN_M = 0.008

# How much TALLER a side head must read before it may override the top head's
# height. Not a preference — without it the arbitration has a positive bias by
# construction and cannot be conservative.
#
# THE DEFECT THIS CLOSES. `fuse_dims_mm` takes height = max(top, side_99.5) and
# degraded to the top only when the side read was <= the top's, i.e. a threshold
# of ZERO. A maximum over a noisy estimator can only ever inflate: any positive
# noise excursion wins and nothing pulls back. Measured cost, on the identical
# cell (pen, seed 1, same pose, same K = 1.0): one head reads 147x11x9, where the
# 9 mm sits under MIN_DIMS_MM and the item is correctly "small" -> C; the rig with
# side heads reads 147x11x11, the item stops being small, and the verdict is then
# decided by SHAPE -> D. The fusion did not measure worse; it lifted the item out
# from under the size rule, and the Pen is the one catalogue item whose correct
# verdict rests on a dimension being SMALL.
#
# WHY THE EXISTING GUARD DID NOT CATCH IT. multiview.SIDE_MIN_POINTS = 30 exists
# for exactly this ("a thin item edge-on ... its z-spread is noise") but counts
# POINTS, and the Pen is 148 mm long: its flank returns points in quantity while
# its 9 mm height is below what the head resolves.
#
# THE BOUND IS THE HEAD'S OWN VERTICAL RESOLUTION, and it is why this is a
# calculation and not a tuned number. Side head at |y| = 0.90 m, z = 0.75 aimed at
# the belt, 320x240, hfov 1.05 rad -> vfov 0.820 rad; range to an item on the belt
# 0.93 m, so ONE PIXEL subtends 3.4 mm vertically. Checked against a real dumped
# side frame rather than trusted: the 200 mm flank of Box 300x200x200 spans 48
# rows = 4.2 mm per row. So a 9 mm item is 2-3 pixels tall and its observed 9->11
# inflation is 0.6 of a pixel. 5 mm is the smallest value above that floor, and it
# is still OPTIMISTIC against the sensor: our sim noise is sigma 3 mm while a
# D435-class datasheet is 1-2 % of range, i.e. 10-20 mm at this distance.
#
# WHAT IT REVERSES, stated plainly because the tree says the opposite in two
# places: SIDE_BELT_MARGIN_M above and tests/test_multiview.py both record that
# "the 9 mm pen is why the heads are there". That was an untested assumption. The
# top head measures the Pen correctly at 9 mm; the side head is the one that gets
# it wrong, because 9 mm is under its resolution. The heads keep their real job —
# the Helmet dome and any hidden vertical extent, gains of TENS of mm, which clear
# this threshold by an order of magnitude.
SIDE_HEIGHT_MIN_GAIN_MM = 5.0

# How many frame periods a side head may lag before its frame is discarded rather
# than compensated. Compensation corrects the belt TRANSLATION; it cannot correct
# what rotated or settled in between, so a badly lagged frame is not slightly
# wrong but describing a different piece of belt. Two periods is 133 ms and, at
# 1 m/s, 133 mm of travel — already far outside the 5 mm accuracy budget.
CAMERA_SIDE_STALE_FRAMES = 2.0
