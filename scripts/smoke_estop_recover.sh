#!/usr/bin/env bash
# Runtime safety smoke, RECOVERY edition: after an E-stop the cell must come
# back on an explicit False — soft-start re-arms, the frozen item resumes and
# still reaches its terminal, and the cleared jam/feed anchors do not re-latch
# the cell on the first frame after the reset (controller_node.on_emergency_stop
# clears them precisely because the standstill is baked into the old anchors).
#
#   bash scripts/smoke_estop_recover.sh
#
# Passes when: the item moves, the E-stop freezes it, the False reset logs a
# second soft-start, the item moves again and is carried through the divert
# section into the B end, and the E-stop never re-latches on its own.
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

# Same controller vocabulary as run_skeleton.sh — the diverter world speaks
# angles, not speeds (see smoke_estop.sh for the trap this avoids).
export HOLD_S=${HOLD_S:-2.5}
export FIRE_LEAD_S=${FIRE_LEAD_S:-0.5}
export ENGAGE_CMD=${ENGAGE_CMD:-0.90}
export RETRACT_CMD=${RETRACT_CMD:-0.0}
export ESTOP_HOLD_MECHANISM=${ESTOP_HOLD_MECHANISM:-true}

LOGDIR=${LOGDIR:-runs/smoke_estop_recover_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$LOGDIR"

cleanup() {
    kill "${LAUNCH:-}" 2>/dev/null || true
    pkill -f "skeleton.launch" 2>/dev/null || true
    pkill -f "src\..*_node" 2>/dev/null || true
    pkill -f parameter_bridge 2>/dev/null || true
    pkill -f "ign gazebo" 2>/dev/null || true
}
trap cleanup EXIT
cleanup
sleep 2

ign gazebo -s -r -v 0 sim/worlds/cell_diverter.sdf > "$LOGDIR/gazebo.log" 2>&1 &
sleep 10
# Start from the negative joint limit, as the normal end-to-end runner does;
# the finite prismatic belt needs the full stroke to carry the item to B.
ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: -3.0" > /dev/null
sleep 4
ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: 0" > /dev/null
sleep 1
ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$PWD/sim/models/items/box_300x200x200/model.sdf\", name: \"item\", pose: {position: {x: -1.0, y: 0, z: 0.5}}" > /dev/null
sleep 2

ros2 launch launch/skeleton.launch.py > "$LOGDIR/skeleton.log" 2>&1 &
LAUNCH=$!
# Let the BOOT soft-start finish before the scenario starts: the E-stop must
# land on a belt at full speed, and the "one soft-start per start" count below
# relies on the boot ramp having logged its completion.
for _ in $(seq 1 60); do
    grep -q "soft-start done" "$LOGDIR/skeleton.log" && break
    sleep 0.5
done
grep -q "soft-start done" "$LOGDIR/skeleton.log" \
    || { echo "FAIL: belt never soft-started at boot"; exit 1; }

item_x() {
    local x=""
    for _ in 1 2 3; do
        x=$(ign model -m item --pose 2>/dev/null | grep -A1 "XYZ" | tail -1 \
            | tr -d "[]" | awk '{print $1}')
        [ -n "$x" ] && break
        sleep 0.3
    done
    echo "${x:-nan}"
}

wait_for_motion() {  # echoes "x1 x2" of the succeeding interval, rc=1 on budget
    for _ in $(seq 1 30); do
        local x1 x2
        x1=$(item_x)
        sleep 1
        x2=$(item_x)
        if python3 -c "assert float('$x2') - float('$x1') > 0.03" 2>/dev/null; then
            echo "$x1 $x2"
            return 0
        fi
    done
    return 1
}

wait_for_motion > /dev/null || { echo "FAIL: item never started moving"; exit 1; }

ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}" > /dev/null

# Ignore the short physical coast immediately after stop, then measure a
# settled interval — the same pattern as smoke_estop.sh.
sleep 0.8
X_STOPPED_1=$(item_x)
sleep 1.2
X_STOPPED_2=$(item_x)
python3 -c "
s1, s2 = float('$X_STOPPED_1'), float('$X_STOPPED_2')
print(f'after E-stop: dx={abs(s2 - s1):.3f} m / 1.2 s')
assert abs(s2 - s1) < 0.05, 'FAIL: item kept moving after E-stop'
"

ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: false}" > /dev/null

# The reset must re-arm the ramp: one soft-start from boot, a second from the
# reset. Wait on the LOG, not wall-clock — the ramp runs on sim time.
RESTARTED=0
for _ in $(seq 1 60); do
    if [ "$(grep -c 'soft-start done' "$LOGDIR/skeleton.log")" -ge 2 ]; then
        RESTARTED=1
        break
    fi
    sleep 0.5
done
grep -q "E-STOP reset" "$LOGDIR/skeleton.log" \
    || { echo "FAIL: controller never acknowledged the reset"; exit 1; }
[ "$RESTARTED" = 1 ] || { echo "FAIL: belt never soft-started after the reset"; exit 1; }

MOVED=$(wait_for_motion) || { echo "FAIL: item did not resume after the reset"; exit 1; }
echo "resumed: x $MOVED"

# The B item must still complete its transport: carried through the camera
# section and past both parked blades (x>=2.4/2.9) into the B end. This smoke
# has no verdict poll that would stop the belt on arrival (that is the episode
# runners' job), so the slab keeps carrying the item beyond the zone afterwards
# — the check is "crossed into the B end", not a zone placement verdict.
DELIVERED=0
for _ in $(seq 1 60); do
    X_NOW=$(item_x)
    if python3 -c "assert float('$X_NOW') > 3.3" 2>/dev/null; then
        DELIVERED=1
        break
    fi
    sleep 1
done
[ "$DELIVERED" = 1 ] || { echo "FAIL: item never crossed into the B end (x=$(item_x))"; exit 1; }

# The cleared anchors must not re-latch the cell: exactly one E-stop, the
# commanded one — no jam/feed watchdog firing off the standstill.
python3 -c "
log = open('$LOGDIR/skeleton.log', encoding='utf-8').read()
assert log.count('E-STOP active') == 1, 'FAIL: the cell re-latched after the reset'
"
echo "PASS: E-stop froze the item; reset soft-started the belt and transport completed into the B end"
echo "logs: $LOGDIR"
