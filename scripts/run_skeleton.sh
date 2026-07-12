#!/usr/bin/env bash
# End-to-end skeleton run (day 3, v0.1-skeleton): spawn an item, let the full
# loop route it (camera -> perception -> classifier -> controller -> pusher),
# check it reached its zone. NO manual commands in the loop.
#
#   bash scripts/run_skeleton.sh <slug> <B|C|D> [spawn_x]
#
# Reproducibility: the spawn position is fixed (x=spawn_x, y=0; default x=-1.5 —
# inside the infeed guide rails so the belt ramp completes before the camera
# window, see belt_guides in cell.sdf) and the world restarts per episode.
# Orientation is the identity by default (default STL pose); scripts/run_matrix.sh
# overrides it per cell via ORIENT_{X,Y,Z,W} env vars from a seeded generator
# (scripts/spawn_orientations.py) — that is the run's seed source (day 4).
#
# Zone success criteria: scripts/zone_verdict.py (shared with run_stream.sh).
cd "$(dirname "$0")/.."
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
ROS_INSTALL_ROOT=${ROS_INSTALL_ROOT:-install}
source "$ROS_INSTALL_ROOT/setup.bash"
set -e

# Every matrix cell starts a fresh Gazebo server.  A plain TERM-only `pkill`
# occasionally left the server alive after an early `set -e` exit (for example,
# a timed-out create service).  The next cell then talked to competing
# `/world/cell/create` services and produced a false TIMEOUT.  Keep cleanup on
# EXIT as well as on the normal path, wait for TERM, and escalate only stale
# processes that did not stop.
LAUNCH=""
DYN_PID=""
GAZEBO_PATTERN='^ign gazebo( |$)'
cleanup() {
    set +e
    [ -n "$DYN_PID" ] && kill "$DYN_PID" 2>/dev/null
    [ -n "$LAUNCH" ] && kill "$LAUNCH" 2>/dev/null
    pkill -f "skeleton.launch" 2>/dev/null || true
    pkill -f "src\..*_node" 2>/dev/null || true
    pkill -f parameter_bridge 2>/dev/null || true
    pkill -TERM -f "$GAZEBO_PATTERN" 2>/dev/null || true
    for _ in {1..20}; do
        pgrep -f "$GAZEBO_PATTERN" >/dev/null 2>&1 || return 0
        sleep 0.1
    done
    pkill -KILL -f "$GAZEBO_PATTERN" 2>/dev/null || true
    for _ in {1..20}; do
        pgrep -f "$GAZEBO_PATTERN" >/dev/null 2>&1 || return 0
        sleep 0.1
    done
}
trap cleanup EXIT

SLUG=${1:?usage: run_skeleton.sh <slug> <B|C|D> [spawn_x]}
EXPECT=${2:?expected zone B|C|D}
SPAWN_X=${3:--1.5}
# Spawn HEIGHT is not a constant: the model's origin is its default-pose bottom
# and Gazebo rotates about it, so a fixed z=0.5 buries turned/tall items inside
# the belt (top surface z=0.4) and the solver ejects them at the spawn (feed_jam,
# seed-0 census). run_matrix.sh computes it per cell from the item's true lowest
# point in that orientation (spawn_orientations.spawn_height_m).
SPAWN_Z=${SPAWN_Z:-0.5}
# Mechanism seam: default = ballistic pusher (cell.sdf). scripts/compare_mechanisms.sh
# swaps in the diverter world; the command topics are identical so nodes are unchanged.
WORLD=${WORLD:-sim/worlds/cell.sdf}
# Generated organizer models are intentionally gitignored. CI points this seam
# at a text-only geometric fixture with the same domain dimensions and mass.
ITEM_MODEL_ROOT=${ITEM_MODEL_ROOT:-sim/models/items}
# Diverter semantics on those same topics (forwarded by skeleton.launch.py as
# controller parameters): the blade is a WALL — it must finish forming BEFORE
# the item's front edge enters its sweep zone (FIRE_LEAD_S; pusher timing
# smacks the item mid-swing, measured 134 m/s^2) and stay engaged while the
# belt slides the item off its edge (HOLD_S; a 0.6 s pusher stroke drops the
# wall in front of the item).
# The blade is POSITION-driven (rad), not velocity-driven: held against its 0.95
# limit by a velocity command it stopped accepting commands and never returned
# (day 10). ENGAGE_CMD is the engaged angle — just inside the limit, so the PID
# never sits on the hard stop — and RETRACT_CMD is the parked angle.
case "$WORLD" in *diverter*)
    export HOLD_S=${HOLD_S:-2.5}
    export FIRE_LEAD_S=${FIRE_LEAD_S:-0.5}
    export ENGAGE_CMD=${ENGAGE_CMD:-0.90}
    export RETRACT_CMD=${RETRACT_CMD:-0.0}
;; esac
# Gentleness metric seam (opt-in): dump the item's dynamic pose during the episode
# and print a peak speed/accel/impulse line. Off by default so the matrix and the
# pusher baseline are not slowed; compare_mechanisms.sh turns it on.
CAPTURE_DYNAMICS=${CAPTURE_DYNAMICS:-0}
# Spawn orientation quaternion (identity by default); run_matrix.sh sets these.
OX=${ORIENT_X:-0}
OY=${ORIENT_Y:-0}
OZ=${ORIENT_Z:-0}
OW=${ORIENT_W:-1}

cleanup

ign gazebo -s -r -v 0 "$WORLD" > /tmp/gz_e2e.log 2>&1 &
sleep 10

# Pre-roll the belt against its LOWER joint limit: the prismatic slab's stroke is
# the episode's fuel (it stops dead at the limit), so every run must start with
# the full stroke ahead of it. Reverse at 3 m/s — faster than belt speed, nothing
# is on the belt yet — so the joint reaches the limit and PINS there within these
# 4 s in either world (cell.sdf: -3.2 m, cell_diverter.sdf: -10 m). At 1 m/s the
# old -1.0 command left the slab wherever the sleep ended, which only happened to
# be the limit because the stroke was short.
ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: -3.0" > /dev/null
sleep 4
ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: 0" > /dev/null
sleep 1

# spawn BEFORE the nodes start: the item rides the controller's soft-start
# ramp from x=SPAWN_X inside the infeed guide rails and reaches the camera
# window (x ~0.9) already settled at full belt speed
ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$PWD/$ITEM_MODEL_ROOT/$SLUG/model.sdf\", name: \"item\", pose: {position: {x: $SPAWN_X, y: 0, z: $SPAWN_Z}, orientation: {x: $OX, y: $OY, z: $OZ, w: $OW}}" > /dev/null
sleep 2

# record the item's dynamic pose for the whole episode (gentleness metric)
if [ "$CAPTURE_DYNAMICS" = 1 ]; then
    ign topic -e -t /world/cell/dynamic_pose/info > /tmp/dyn_trace.log 2>&1 &
    DYN_PID=$!
fi

T0=$(date +%s.%N)
ros2 launch launch/skeleton.launch.py > /tmp/skeleton_e2e.log 2>&1 &
LAUNCH=$!

item_pose() {  # x y z, with retries against ign CLI flakes
    local out=""
    for _ in 1 2 3; do
        out=$(ign model -m item --pose 2>/dev/null | grep -A1 "XYZ" | tail -1 \
              | tr -d "[]" | awk '{print $1, $2, $3}')
        [ -n "$out" ] && break
        sleep 0.5
    done
    echo "${out:-nan nan nan}"
}

# verdict polling is WALL-clock; under render load (record_skeleton_video.sh)
# RTF drops ~10x, so the recorder widens the window via this env override
POLL_ITERS=${RUN_SKELETON_POLL_ITERS:-60}

VERDICT=FAIL
for _ in $(seq 1 "$POLL_ITERS"); do
    read X Y Z <<< "$(item_pose)"
    OK=$(python3 scripts/zone_verdict.py "$EXPECT" "$X" "$Y" "$Z")
    if [ "$OK" = YES ]; then
        VERDICT=PASS
        break
    fi
    sleep 0.5
done
T1=$(date +%s.%N)
CYCLE=$(python3 -c "print(f'{$T1 - $T0:.1f}')")

read X Y Z <<< "$(item_pose)"
echo "$SLUG -> $EXPECT: $VERDICT (pose x=$X y=$Y z=$Z, cycle ${CYCLE}s from launch)"
grep -E "item [0-9]+:" /tmp/skeleton_e2e.log | tail -3 || true

if [ -n "$DYN_PID" ]; then
    kill "$DYN_PID" 2>/dev/null || true
    MASS=$(grep -m1 '<mass>' "$ITEM_MODEL_ROOT/$SLUG/model.sdf" \
           | sed -E 's/.*<mass>([0-9.]+)<.*/\1/')
    python3 scripts/capture_dynamics.py /tmp/dyn_trace.log --mass "${MASS:-1.0}" || true
fi

[ "$VERDICT" = PASS ]
