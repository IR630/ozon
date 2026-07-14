#!/usr/bin/env bash
# Runtime safety smoke: prove that /emergency_stop stops a moving Gazebo item
# and emits zero commands for the belt and both mechanisms.
# Run from the repository root in the ROS 2 / Gazebo environment:
#   bash scripts/smoke_estop.sh
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

# This smoke runs the DIVERTER world, so the controller must speak the diverter's
# vocabulary (angles, not speeds) exactly as run_skeleton.sh sets it — otherwise a
# fire would publish 2.5 as an ANGLE. Nothing fires here (the E-stop lands first),
# but leaving the mismatch in place is a trap for whoever extends this file.
export HOLD_S=${HOLD_S:-2.5}
export FIRE_LEAD_S=${FIRE_LEAD_S:-0.5}
export ENGAGE_CMD=${ENGAGE_CMD:-0.90}
export RETRACT_CMD=${RETRACT_CMD:-0.0}
# The blade is parked when this smoke stops the cell, so freezing it and zeroing it
# are the same command (0.0) — the probe below sees a zero either way. The BUSY-cell
# case, where the difference is a blade swinging out from under an item, is
# scripts/smoke_estop_stream.sh.
export ESTOP_HOLD_MECHANISM=${ESTOP_HOLD_MECHANISM:-true}

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

ign gazebo -s -r -v 0 sim/worlds/cell_diverter.sdf > /tmp/gz_estop.log 2>&1 &
sleep 10
# Start from the negative joint limit, as the normal end-to-end runner does;
# otherwise the finite prismatic belt can reach +3.2 m before the stop sample.
ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: -1.0" > /dev/null
sleep 4
ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: 0" > /dev/null
sleep 1
ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$PWD/sim/models/items/box_300x200x200/model.sdf\", name: \"item\", pose: {position: {x: -1.0, y: 0, z: 0.5}}" > /dev/null
sleep 2

ros2 launch launch/skeleton.launch.py > /tmp/skeleton_estop.log 2>&1 &
LAUNCH=$!

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

# Rendering load changes Gazebo's real-time factor. Wait for measured motion
# instead of assuming a wall-clock delay for the sim-time soft-start timer.
MOVING=0
for _ in $(seq 1 30); do
    X_MOVING_1=$(item_x)
    sleep 1
    X_MOVING_2=$(item_x)
    if python3 -c "assert float('$X_MOVING_2') - float('$X_MOVING_1') > 0.03" \
            2>/dev/null; then
        MOVING=1
        break
    fi
done
[ "$MOVING" = 1 ] || { echo "FAIL: item never started moving"; exit 1; }

# One rclpy process observes all three topics without the shared ros2 CLI daemon.
python3 scripts/probe_estop.py > /tmp/estop_commands.log &
PROBE=$!
sleep 1
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}" > /dev/null
wait "$PROBE"

# Ignore the short physical coast immediately after stop, then measure a
# settled interval. The item must have moved before E-stop and remain still now.
sleep 0.8
X_STOPPED_1=$(item_x)
sleep 1.2
X_STOPPED_2=$(item_x)

python3 -c "
x1, x2 = float('$X_MOVING_1'), float('$X_MOVING_2')
s1, s2 = float('$X_STOPPED_1'), float('$X_STOPPED_2')
moving_delta, stopped_delta = x2 - x1, abs(s2 - s1)
print(f'before E-stop: dx={moving_delta:.3f} m / 1.0 s')
print(f'after E-stop:  dx={stopped_delta:.3f} m / 1.2 s')
assert moving_delta > 0.03, 'FAIL: item was not moving before E-stop'
assert stopped_delta < 0.05, 'FAIL: item kept moving after E-stop'
"

grep -q "zero commands observed" /tmp/estop_commands.log
grep -q "E-STOP active" /tmp/skeleton_estop.log
echo "PASS: E-stop halted the moving item and zeroed belt, C and D commands"
