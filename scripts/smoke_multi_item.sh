#!/usr/bin/env bash
# Full-contour multi-item smoke: two products share one camera frame, keep
# distinct IDs, and independently reach B in the final diverter world.
# errexit first, and a loud check before sourcing install/setup.bash — same
# defect/fix as run_skeleton.sh (see that file's header for the full story).
set -e
cd "$(dirname "$0")/.."

export LIBGL_ALWAYS_SOFTWARE=1
ROS_INSTALL_ROOT=${ROS_INSTALL_ROOT:-install}
if [ ! -f "$ROS_INSTALL_ROOT/setup.bash" ]; then
    echo "ABORT: ROS workspace is not built ($ROS_INSTALL_ROOT/setup.bash is missing) — run:" >&2
    echo "    colcon build --packages-select ros_msgs" >&2
    exit 1
fi
source /opt/ros/humble/setup.bash
source "$ROS_INSTALL_ROOT/setup.bash"
# nounset only AFTER the sources — never before. ROS's own setup.bash reads
# unbound variables (`AMENT_TRACE_SETUP_FILES: unbound variable`) and dies under
# `set -u`, taking the whole script with it on a perfectly good workspace. This
# is exactly where `set -eu` always sat, so -u's scope is unchanged; only errexit
# moved up to cover the sources.
set -u

cleanup() {
    kill "${PROBE:-}" "${LAUNCH:-}" 2>/dev/null || true
    pkill -f "skeleton.launch" 2>/dev/null || true
    pkill -f "src\\..*_node" 2>/dev/null || true
    pkill -f parameter_bridge 2>/dev/null || true
    pkill -f "ign gazebo" 2>/dev/null || true
}
trap cleanup EXIT
cleanup
sleep 2

WORLD=sim/worlds/cell_diverter.sdf
MODEL_ROOT=${ITEM_MODEL_ROOT:-sim/models/items}
MODEL="$PWD/$MODEL_ROOT/box_300x200x200/model.sdf"

ign gazebo -s -r -v 0 "$WORLD" > /tmp/gz_multi_item.log 2>&1 &
sleep 10
ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: -1.0" > /dev/null
sleep 4
ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: 0" > /dev/null
sleep 1

spawn() {
    local name=$1 x=$2
    ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
        --reptype ignition.msgs.Boolean --timeout 5000 \
        --req "sdf_filename: \"$MODEL\", name: \"$name\", pose: {position: {x: $x, y: 0, z: 0.405}}" > /dev/null
}
spawn item_front -1.0
spawn item_rear -1.65
sleep 2

export HOLD_S=2.5
export FIRE_LEAD_S=0.5
ros2 launch launch/skeleton.launch.py > /tmp/skeleton_multi_item.log 2>&1 &
LAUNCH=$!
python3 scripts/probe_multi_item.py > /tmp/probe_multi_item.log 2>&1 &
PROBE=$!

item_pose() {
    local name=$1 out=""
    for _ in 1 2 3; do
        out=$(ign model -m "$name" --pose 2>/dev/null | grep -A1 "XYZ" | tail -1 \
            | tr -d "[]" | awk '{print $1, $2, $3}')
        [ -n "$out" ] && break
        sleep 0.3
    done
    echo "${out:-nan nan nan}"
}

BOTH_IN_B=0
for _ in $(seq 1 100); do
    read X1 Y1 Z1 <<< "$(item_pose item_front)"
    read X2 Y2 Z2 <<< "$(item_pose item_rear)"
    if python3 -c "
x1, z1, x2, z2 = map(float, ('$X1', '$Z1', '$X2', '$Z2'))
assert x1 >= 3.5 and 0.35 <= z1 <= 1.0
assert x2 >= 3.5 and 0.35 <= z2 <= 1.0
" 2>/dev/null; then
        BOTH_IN_B=1
        break
    fi
    sleep 0.5
done
[ "$BOTH_IN_B" = 1 ] || { echo "FAIL: both items did not reach B"; exit 1; }
wait "$PROBE"

read X1 Y1 Z1 <<< "$(item_pose item_front)"
read X2 Y2 Z2 <<< "$(item_pose item_rear)"
cat /tmp/probe_multi_item.log
echo "item_front pose: x=$X1 y=$Y1 z=$Z1"
echo "item_rear  pose: x=$X2 y=$Y2 z=$Z2"
grep -q "item 1: B" /tmp/skeleton_multi_item.log
grep -q "item 2: B" /tmp/skeleton_multi_item.log
echo "PASS: two simultaneous items kept distinct IDs and independently reached B"
