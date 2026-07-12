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
# Zone success criteria (world frame, the union over both mechanism worlds —
# the pusher drops the item at its paddle x (cell.sdf patches at C=2.5, D=3.0),
# the diverter funnels it to its pivot x (cell_diverter.sdf patches at C=3.0,
# D=3.5), so the x bands span both; day-4 retro also asked for wider tolerances
# (box_400 was routed correctly but FAILed by millimetres):
#   B: rode past the pushers on the belt (x >= 3.5, still at belt height)
#   C: landed in the zone C roll-cage (x 1.9..3.6, y 0.5..1.4, on the floor)
#   D: landed in the zone D roll-cage (x 2.4..4.1, y -1.4..-0.5, on the floor)
# The y/z bounds carry cage slack over the flat patch footprint (patch edge
# y=1.3/-1.3, physical cage wall at y=1.5/-1.5; z<0.25 clears a 400 mm-tall box
# standing on its base at center z~0.2 yet stays well under belt height z~0.45).
# B stays unambiguous: C/D require the floor (z<0.25), B requires belt height.
cd "$(dirname "$0")/.."
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
ROS_INSTALL_ROOT=${ROS_INSTALL_ROOT:-install}
source "$ROS_INSTALL_ROOT/setup.bash"
set -e

SLUG=${1:?usage: run_skeleton.sh <slug> <B|C|D> [spawn_x]}
EXPECT=${2:?expected zone B|C|D}
SPAWN_X=${3:--1.5}
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
case "$WORLD" in *diverter*)
    export HOLD_S=${HOLD_S:-2.5}
    export FIRE_LEAD_S=${FIRE_LEAD_S:-0.5}
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

pkill -f "ign gazebo" 2>/dev/null || true
pkill -f "skeleton.launch" 2>/dev/null || true
pkill -f "src\..*_node" 2>/dev/null || true
pkill -f parameter_bridge 2>/dev/null || true
sleep 2

ign gazebo -s -r -v 0 "$WORLD" > /tmp/gz_e2e.log 2>&1 &
sleep 10

# pre-roll the belt to its -3.2 m joint limit: full 6.4 m of travel for the run
ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: -1.0" > /dev/null
sleep 4
ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: 0" > /dev/null
sleep 1

# spawn BEFORE the nodes start: the item rides the controller's soft-start
# ramp from x=SPAWN_X inside the infeed guide rails and reaches the camera
# window (x ~0.9) already settled at full belt speed
ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$PWD/$ITEM_MODEL_ROOT/$SLUG/model.sdf\", name: \"item\", pose: {position: {x: $SPAWN_X, y: 0, z: 0.5}, orientation: {x: $OX, y: $OY, z: $OZ, w: $OW}}" > /dev/null
sleep 2

# record the item's dynamic pose for the whole episode (gentleness metric)
DYN_PID=""
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
    OK=$(python3 -c "
x, y, z = float('$X'), float('$Y'), float('$Z')
checks = {
    'B': x >= 3.5 and 0.35 <= z <= 1.0,
    'C': 1.9 <= x <= 3.8 and 0.5 <= y <= 1.4 and z < 0.25,
    'D': 2.4 <= x <= 4.3 and -1.4 <= y <= -0.5 and z < 0.25,
}
print('YES' if checks['$EXPECT'] else 'no')
")
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

kill $LAUNCH 2>/dev/null || true
sleep 1
pkill -f "skeleton.launch" 2>/dev/null || true
pkill -f "src\..*_node" 2>/dev/null || true
pkill -f parameter_bridge 2>/dev/null || true
pkill -f "ign gazebo" 2>/dev/null || true
[ "$VERDICT" = PASS ]
