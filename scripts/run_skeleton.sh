#!/usr/bin/env bash
# End-to-end skeleton run (day 3, v0.1-skeleton): spawn an item, let the full
# loop route it (camera -> perception -> classifier -> controller -> pusher),
# check it reached its zone. NO manual commands in the loop.
#
#   bash scripts/run_skeleton.sh <slug> <B|C|D> [spawn_x]
#
# Reproducibility: the spawn pose is fixed (x=spawn_x, y=0, default STL
# orientation) and the world restarts per episode — the run is deterministic
# up to physics chaos; there is no randomness to seed yet (seed enters with
# random orientations on day 4).
#
# Zone success criteria (world frame, mirrors sim/worlds/cell.sdf):
#   B: rode past the pushers on the belt (x >= 3.5, still at belt height)
#   C: landed on the zone C patch (x 1.9..3.1, y 0.5..1.3, on the floor)
#   D: landed on the zone D patch (x 2.4..3.6, y -1.3..-0.5, on the floor)
cd "$(dirname "$0")/.."
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
source install/setup.bash
set -e

SLUG=${1:?usage: run_skeleton.sh <slug> <B|C|D> [spawn_x]}
EXPECT=${2:?expected zone B|C|D}
SPAWN_X=${3:-0.0}

pkill -f "ign gazebo" 2>/dev/null || true
pkill -f "skeleton.launch" 2>/dev/null || true
pkill -f "src\..*_node" 2>/dev/null || true
pkill -f parameter_bridge 2>/dev/null || true
sleep 2

ign gazebo -s -r -v 0 sim/worlds/cell.sdf > /tmp/gz_e2e.log 2>&1 &
sleep 10

# pre-roll the belt to its -3.2 m joint limit: full 6.4 m of travel for the run
ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: -1.0" > /dev/null
sleep 4
ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: 0" > /dev/null
sleep 1

# spawn BEFORE the nodes start: the item rides the controller's soft-start
# ramp from x=SPAWN_X; the camera picks it up around x=0.8
ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$PWD/sim/models/items/$SLUG/model.sdf\", name: \"item\", pose: {position: {x: $SPAWN_X, y: 0, z: 0.5}}" > /dev/null
sleep 2

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

VERDICT=FAIL
for _ in $(seq 1 60); do
    read X Y Z <<< "$(item_pose)"
    OK=$(python3 -c "
x, y, z = float('$X'), float('$Y'), float('$Z')
checks = {
    'B': x >= 3.5 and 0.35 <= z <= 1.0,
    'C': 1.9 <= x <= 3.1 and 0.5 <= y <= 1.3 and z < 0.2,
    'D': 2.4 <= x <= 3.6 and -1.3 <= y <= -0.5 and z < 0.2,
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

kill $LAUNCH 2>/dev/null || true
sleep 1
pkill -f "skeleton.launch" 2>/dev/null || true
pkill -f "src\..*_node" 2>/dev/null || true
pkill -f parameter_bridge 2>/dev/null || true
pkill -f "ign gazebo" 2>/dev/null || true
[ "$VERDICT" = PASS ]
