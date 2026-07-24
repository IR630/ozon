#!/usr/bin/env bash
# Runtime safety smoke, STREAM edition (day 10): an E-stop must be safe while the
# cell is BUSY — several items on the belt and a blade already engaged as a wall.
#
# scripts/smoke_estop.sh proves the single-item case: one item, no mechanism
# engaged, everything goes to zero. That case cannot catch the failure this one
# exists for: on the POSITION-driven diverter a 0.0 command does not mean "stop",
# it means "return to the parked angle" — so zeroing the topic on E-stop would
# SWING an engaged blade home, out from under the item leaning against it. Motion
# during an emergency stop. The blade must freeze exactly where it is.
#
#   bash scripts/smoke_estop_stream.sh
#
# Passes when, after the E-stop: the items stop moving, the blade's ANGLE does not
# change (measured on the joint, not assumed from the command), and no item fires.
# Then the operator-clear/reset half removes the stopped goods, proves that the
# controller's fresh-feedback gate accepts both parked blades, re-reads the real
# joint after that gate, and sends a fresh B item straight past both diverters
# without another actuation.
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

# The occupied-stop scenario needs the C blade to remain a wall until the stop
# arrives. This smoke spends several seconds on two joint reads and first-publish
# discovery, whereas the production 2.5 s hold may retract normally before the
# E-stop is sent. A longer value affects only this test setup, not the production
# default in controller_node.py; the stop is still sent immediately after engage.
export HOLD_S=${HOLD_S:-15.0}
export FIRE_LEAD_S=${FIRE_LEAD_S:-0.5}
export ENGAGE_CMD=${ENGAGE_CMD:-0.90}
export RETRACT_CMD=${RETRACT_CMD:-0.0}
export ESTOP_HOLD_MECHANISM=${ESTOP_HOLD_MECHANISM:-true}
ITEM_MODEL_ROOT=${ITEM_MODEL_ROOT:-sim/models/items}
LOGDIR=${LOGDIR:-runs/estop_stream_$(date +%Y%m%d_%H%M%S)}

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
mkdir -p "$LOGDIR"

ign gazebo -s -r -v 0 sim/worlds/cell_diverter.sdf > "$LOGDIR/gazebo.log" 2>&1 &
sleep 10
ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: -3.0" > /dev/null
sleep 4
ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: 0" > /dev/null
sleep 1

ros2 launch launch/skeleton.launch.py > "$LOGDIR/skeleton.log" 2>&1 &
LAUNCH=$!
for _ in $(seq 1 60); do
    grep -q "soft-start done" "$LOGDIR/skeleton.log" && break
    sleep 0.5
done
grep -q "soft-start done" "$LOGDIR/skeleton.log" \
    || { echo "FAIL: controller never completed its initial soft-start"; exit 1; }

spawn() {  # slug name
    ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
        --reptype ignition.msgs.Boolean --timeout 5000 \
        --req "sdf_filename: \"$PWD/$ITEM_MODEL_ROOT/$1/model.sdf\", name: \"$2\", pose: {position: {x: -1.5, y: 0, z: 0.405}}" > /dev/null
}
remove_model() {  # name
    local reply
    reply=$(ign service -s /world/cell/remove --reqtype ignition.msgs.Entity \
        --reptype ignition.msgs.Boolean --timeout 5000 \
        --req "name: \"$1\", type: 2")
    grep -Eq "data:[[:space:]]*true" <<< "$reply" \
        || { echo "FAIL: Gazebo did not remove $1: $reply"; exit 1; }
}
# Two items to the SAME zone: the blade engages for the first and is still a wall
# when the E-stop lands — exactly the state the single-item smoke cannot reach.
spawn box_400x400x300 item0
sleep 1
spawn box_400x400x300 item1

item_pose() {
    ign model -m "$1" --pose 2>/dev/null | grep -A1 "XYZ" | tail -1 \
        | tr -d "[]" | awk '{print $1, $2, $3}'
}
blade_angle() {  # C blade, read from the JOINT — not from the command we sent
    timeout 2 ign topic -e -t /world/cell/model/diverter_c/joint_state 2>/dev/null \
        | python3 scripts/parse_ign_joint_angle.py || true
}

# Wait until the blade is actually engaged: that is the state under test.
ENGAGED=0
for _ in $(seq 1 60); do
    A=$(blade_angle)
    if [ -n "${A:-}" ] && python3 -c "import sys; sys.exit(0 if float('$A') > 0.5 else 1)"; then
        ENGAGED=1
        break
    fi
    sleep 0.5
done
[ "$ENGAGED" = 1 ] || { echo "FAIL: the blade never engaged — nothing to E-stop"; exit 1; }

A_BEFORE=$(blade_angle)

ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}" > /dev/null
for _ in $(seq 1 30); do
    grep -q "E-STOP active" "$LOGDIR/skeleton.log" && break
    sleep 0.1
done
grep -q "E-STOP active" "$LOGDIR/skeleton.log" \
    || { echo "FAIL: controller did not process the E-stop"; exit 1; }
# A fire that raced immediately BEFORE the controller processed the E-stop is
# not a post-stop violation. From this log boundary onward, no new fire may occur.
FIRED_AT_STOP=$(grep -c "FIRED" "$LOGDIR/skeleton.log" || true)

# An E-stop cannot cancel physics: an item already sliding down the chute keeps
# going under gravity, and a heavy item leaning on the blade keeps pressing it.
# So allow the coast, then measure a SETTLED interval — the same pattern the
# single-item smoke uses. The blade's 0.4 s swing is long over by then: a blade
# that was being parked would be home.
sleep 3
P0_1=$(item_pose item0); P1_1=$(item_pose item1)
sleep 2
P0_2=$(item_pose item0); P1_2=$(item_pose item1)
A_AFTER=$(blade_angle)
FIRED_AFTER=$(grep -c "FIRED" "$LOGDIR/skeleton.log" || true)

echo "settled pose item0: [$P0_1] -> [$P0_2]"
echo "settled pose item1: [$P1_1] -> [$P1_2]"
echo "blade C angle: $A_BEFORE -> $A_AFTER rad (parked = 0.0)"
echo "FIRED lines after E-stop activation: $FIRED_AT_STOP -> $FIRED_AFTER"

python3 - <<PY
import math

p0_before = tuple(map(float, "$P0_1".split()))
p0_after = tuple(map(float, "$P0_2".split()))
p1_before = tuple(map(float, "$P1_1".split()))
p1_after = tuple(map(float, "$P1_2".split()))
d0 = math.dist(p0_before, p0_after)
d1 = math.dist(p1_before, p1_after)
blade_before = float("$A_BEFORE")
blade_after = float("$A_AFTER")
assert d0 < 0.05, (
    f"FAIL: item0 still moving 3 s after E-stop (d3={d0:.3f} m / 2 s)")
assert d1 < 0.05, (
    f"FAIL: item1 still moving 3 s after E-stop (d3={d1:.3f} m / 2 s)")
# THE safety property: the engaged blade must not be sent home. It may be pushed
# FURTHER open by the item leaning on it (measured: box_400 drives it past its own
# 0.95 limit — the force-controlled blade is compliant, docs/decisions.md), but it
# must never swing back across the item's path.
assert blade_after >= blade_before - 0.05, (
    f"FAIL: blade moved toward park during E-stop "
    f"({blade_before:.3f}->{blade_after:.3f} rad)")
assert $FIRED_AFTER == $FIRED_AT_STOP, "FAIL: an item fired after the E-stop"
PY

# The operator has inspected/cleared the cell. Removing the stopped test goods
# models that manual step; reset is never an automatic jam-clear.
remove_model item0
remove_model item1
SOFT_STARTS_BEFORE=$(grep -c "soft-start done" "$LOGDIR/skeleton.log" || true)
FIRED_AT_RESET=$(grep -c "FIRED" "$LOGDIR/skeleton.log" || true)
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: false}" > /dev/null

for _ in $(seq 1 30); do
    grep -q "parking mechanism C/D before belt soft-start" "$LOGDIR/skeleton.log" && break
    sleep 0.1
done
grep -q "parking mechanism C/D before belt soft-start" "$LOGDIR/skeleton.log" \
    || { echo "FAIL: reset did not start occupied-mechanism recovery"; exit 1; }

# The controller emits this log only after fresh post-command C/D samples entered
# the parking tolerance and before arming its ramp timer. Re-read Gazebo after the
# gate as a physical confirmation; controller logs/tests establish causal order,
# while the fresh terminal-B item below proves the transport path stayed clear.
for _ in $(seq 1 30); do
    grep -q "E-STOP reset: restarting belt with soft-start" \
        "$LOGDIR/skeleton.log" && break
    sleep 0.1
done
grep -q "E-STOP reset: restarting belt with soft-start" "$LOGDIR/skeleton.log" \
    || { echo "FAIL: reset never completed mechanism recovery"; exit 1; }
A_PARKED=$(blade_angle)
[ -n "${A_PARKED:-}" ] \
    || { echo "FAIL: no C joint feedback after occupied recovery"; exit 1; }
python3 -c "
angle = abs(float('$A_PARKED'))
print(f'blade C after recovery gate: {angle:.3f} rad')
assert angle < 0.015, f'FAIL: belt restarted with blade still over the belt ({angle:.3f} rad)'
"

for _ in $(seq 1 60); do
    SOFT_STARTS_NOW=$(grep -c "soft-start done" "$LOGDIR/skeleton.log" || true)
    [ "$SOFT_STARTS_NOW" -gt "$SOFT_STARTS_BEFORE" ] && break
    sleep 0.5
done
[ "${SOFT_STARTS_NOW:-0}" -gt "$SOFT_STARTS_BEFORE" ] \
    || { echo "FAIL: belt never soft-started after occupied reset"; exit 1; }

# A fresh B item is the distinguishing postcondition: a blade left across the
# conveyor would divert it into C. Require the real terminal B band, not merely
# x>3.3 (the D blade still sweeps as far as x=4.15).
B_DECISIONS_BEFORE=$(grep -c "B — rides to belt end" "$LOGDIR/skeleton.log" || true)
spawn box_300x200x200 item2
DELIVERED=0
for _ in $(seq 1 80); do
    POSE2=$(item_pose item2)
    read -r X2 Y2 Z2 <<< "${POSE2:-}"
    VERDICT=$(python3 scripts/zone_verdict.py B \
        "${X2:-nan}" "${Y2:-nan}" "${Z2:-nan}")
    if [ "$VERDICT" = "YES" ]; then
        DELIVERED=1
        break
    fi
    sleep 0.5
done
[ "$DELIVERED" = 1 ] \
    || { echo "FAIL: post-reset item never reached terminal B (pose=${POSE2:-missing})"; exit 1; }
B_DECISIONS_AFTER=$(grep -c "B — rides to belt end" "$LOGDIR/skeleton.log" || true)
[ "$B_DECISIONS_AFTER" -gt "$B_DECISIONS_BEFORE" ] \
    || { echo "FAIL: fresh post-reset item had no new B controller decision"; exit 1; }
FIRED_AFTER_RESET=$(grep -c "FIRED" "$LOGDIR/skeleton.log" || true)
[ "$FIRED_AFTER_RESET" = "$FIRED_AT_RESET" ] \
    || { echo "FAIL: a mechanism fired after occupied reset"; exit 1; }
[ "$(grep -c 'E-STOP active' "$LOGDIR/skeleton.log")" = 1 ] \
    || { echo "FAIL: the cell re-latched during occupied recovery"; exit 1; }

echo "PASS: occupied E-stop froze the cell; feedback gate accepted parked C/D; fresh B crossed straight"
echo "logs: $LOGDIR"
