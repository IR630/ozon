#!/usr/bin/env bash
# The census the camera decision was still missing: ALL THREE rigs under a
# CALIBRATION DRIFT, at the two budgets the offline matrix disagreed across.
#
# WHY THIS EXISTS, AND WHY census_2v3_noise.sh DOES NOT COVER IT. That script runs
# 2 vs 3 heads, clean and noisy. Noise is not the phenomenon the extra heads are
# supposed to lose to — DRIFT is: in Gazebo the heads sit exactly where
# src/constants.py says, so the mutual calibration error is ZERO and every live
# number so far was taken on a rig no integrator will ever own
# (scripts/make_miscal_world.py). The offline matrix (docs/experiments.md 26.07)
# says the rig ORDER INVERTS with that budget — 3A leads 92 % against 82 % at the
# typical column and TRAILS 82-83 % against 86 % in the worst one — so a run at one
# budget cannot decide anything. Both are run here, on the SAME seed.
#
#   bash scripts/census_miscal.sh [seed]        # default seed 0
#
# THE ONE-HEAD RIG IS IN THE TABLE ON PURPOSE, AND IT IS INVARIANT. The drift is
# baked into the SIDE heads only (the top head's calibration is one a one-camera
# rig has to get right too), so the one-head column does not move with the budget.
# That is exactly what makes it the control: the decision rule asks whether fusion
# under drift is WORSE THAN TOP-ONLY, and top-only is the line that does not bend.
#
# Triage each rig dir afterwards with scripts/triage_matrix.py, and read rc=124
# SEPARATELY from routing misses — they are a stopwatch on the machine, not a
# verdict on the optics (docs/decisions.md 28.07, three false conclusions in a day).
set -e
cd "$(dirname "$0")/.."
SEED=${1:-0}
OUT=runs/miscal_$(date +%Y%m%d_%H%M%S)_seed${SEED}
mkdir -p "$OUT"

# Budgets from scripts/probe_camera_count.py CALIBRATIONS, so the live numbers land
# on the offline matrix's columns instead of a third scale nobody can compare to.
TYPICAL_MM=2.0;  TYPICAL_DEG=0.2      # "типичная"
WORST_MM=3.0;    WORST_DEG=0.3        # "посредственная" — the column where 3A lost

echo "=== generating miscalibrated worlds ==="
python3 scripts/make_miscal_world.py \
    sim/worlds/cell_diverter_2cam_miscal.sdf sim/worlds/cell_diverter_2cam.sdf \
    --shift-mm $TYPICAL_MM --tilt-deg $TYPICAL_DEG
python3 scripts/make_miscal_world.py \
    sim/worlds/cell_diverter_2cam_miscal_worst.sdf sim/worlds/cell_diverter_2cam.sdf \
    --shift-mm $WORST_MM --tilt-deg $WORST_DEG
python3 scripts/make_miscal_world.py \
    sim/worlds/cell_diverter_3cam_miscal.sdf sim/worlds/cell_diverter_3cam.sdf \
    --shift-mm $TYPICAL_MM --tilt-deg $TYPICAL_DEG
python3 scripts/make_miscal_world.py \
    sim/worlds/cell_diverter_3cam_miscal_worst.sdf sim/worlds/cell_diverter_3cam.sdf \
    --shift-mm $WORST_MM --tilt-deg $WORST_DEG

# A generated world that does not parse would fail 33 cells one by one and read as
# a broken rig (scripts/check_sdf.sh, trap in the brief).
for w in sim/worlds/cell_diverter_2cam_miscal.sdf \
         sim/worlds/cell_diverter_2cam_miscal_worst.sdf \
         sim/worlds/cell_diverter_3cam_miscal.sdf \
         sim/worlds/cell_diverter_3cam_miscal_worst.sdf; do
    ign sdf -k "$w" > /dev/null || { echo "ABORT: $w does not parse"; exit 1; }
done

# tag             world                                            bridge
CONFIGS=(
    "1cam_control      sim/worlds/cell_diverter.sdf                     bridge.yaml"
    "2cam_typical      sim/worlds/cell_diverter_2cam_miscal.sdf         bridge_2cam.yaml"
    "2cam_worst        sim/worlds/cell_diverter_2cam_miscal_worst.sdf   bridge_2cam.yaml"
    "3cam_typical      sim/worlds/cell_diverter_3cam_miscal.sdf         bridge_3cam.yaml"
    "3cam_worst        sim/worlds/cell_diverter_3cam_miscal_worst.sdf   bridge_3cam.yaml"
)

# HARNESS BUDGETS, not physical limits — see census_2v3_noise.sh for the full story
# (three-head bring-up outruns the shipped 30 s controller wait and every cell then
# aborts with "belt never reached full speed", which reads as a broken rig).
export SOFT_START_TRIES=${SOFT_START_TRIES:-240}
export CELL_TIMEOUT=${CELL_TIMEOUT:-400}

for cfg in "${CONFIGS[@]}"; do
    read -r tag world bridge <<< "$cfg"
    echo "=== $tag ($world / $bridge) $(date -u +%H:%M:%SZ) ==="
    # Per-rig NODE_LOG: heads=N has to be verifiable per run, and a shared path
    # would leave only the last rig's. A rig that silently lost a side head
    # produces a census that looks like a verdict about fusion and is not.
    LOGDIR="$OUT/$tag" WORLD="$world" BRIDGE_CONFIG="$bridge" \
        NODE_LOG="$OUT/$tag.node.log" \
        bash scripts/run_matrix.sh "$SEED" 3 > "$OUT/${tag}.driver.log" 2>&1 || true
    grep -E "routing correctness" "$OUT/$tag/summary.log" 2>/dev/null | tail -1 \
        | sed "s/^/[$tag] /" || echo "[$tag] no summary"
done
echo "=== done -> $OUT ==="
touch "$OUT/DONE"
