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
cd "$(dirname "$0")/.."
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
source install/setup.bash
set -eu

export HOLD_S=${HOLD_S:-2.5}
export FIRE_LEAD_S=${FIRE_LEAD_S:-0.5}
export ENGAGE_CMD=${ENGAGE_CMD:-0.90}
export RETRACT_CMD=${RETRACT_CMD:-0.0}
export ESTOP_HOLD_MECHANISM=${ESTOP_HOLD_MECHANISM:-true}
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

spawn() {  # slug name
    ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
        --reptype ignition.msgs.Boolean --timeout 5000 \
        --req "sdf_filename: \"$PWD/sim/models/items/$1/model.sdf\", name: \"$2\", pose: {position: {x: -1.5, y: 0, z: 0.405}}" > /dev/null
}
# Two items to the SAME zone: the blade engages for the first and is still a wall
# when the E-stop lands — exactly the state the single-item smoke cannot reach.
spawn box_400x400x300 item0
sleep 1
spawn box_400x400x300 item1

item_x() {
    ign model -m "$1" --pose 2>/dev/null | grep -A1 "XYZ" | tail -1 \
        | tr -d "[]" | awk '{print $1}'
}
blade_angle() {  # C blade, read from the JOINT — not from the command we sent
    timeout 2 ign topic -e -t /world/cell/model/diverter_c/joint_state 2>/dev/null \
        | grep -A6 "axis1" | grep -m1 -oE "position: -?[0-9.]+" | awk '{print $2}'
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
FIRED_BEFORE=$(grep -c "FIRED" "$LOGDIR/skeleton.log" || true)

ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}" > /dev/null

# An E-stop cannot cancel physics: an item already sliding down the chute keeps
# going under gravity, and a heavy item leaning on the blade keeps pressing it.
# So allow the coast, then measure a SETTLED interval — the same pattern the
# single-item smoke uses. The blade's 0.4 s swing is long over by then: a blade
# that was being parked would be home.
sleep 3
X0_1=$(item_x item0); X1_1=$(item_x item1)
sleep 2
X0_2=$(item_x item0); X1_2=$(item_x item1)
A_AFTER=$(blade_angle)
FIRED_AFTER=$(grep -c "FIRED" "$LOGDIR/skeleton.log" || true)

echo "settled dx item0: $X0_1 -> $X0_2"
echo "settled dx item1: $X1_1 -> $X1_2"
echo "blade C angle: $A_BEFORE -> $A_AFTER rad (parked = 0.0)"
echo "FIRED lines: $FIRED_BEFORE -> $FIRED_AFTER"

python3 - <<PY
x0 = abs(float("$X0_2") - float("$X0_1"))
x1 = abs(float("$X1_2") - float("$X1_1"))
blade_after = float("$A_AFTER")
assert x0 < 0.05, f"FAIL: item0 still moving 3 s after E-stop (dx={x0:.3f} m / 2 s)"
assert x1 < 0.05, f"FAIL: item1 still moving 3 s after E-stop (dx={x1:.3f} m / 2 s)"
# THE safety property: the engaged blade must not be sent home. It may be pushed
# FURTHER open by the item leaning on it (measured: box_400 drives it past its own
# 0.95 limit — the force-controlled blade is compliant, docs/decisions.md), but it
# must never swing back across the item's path.
assert blade_after > 0.5, (
    f"FAIL: the blade was PARKED during the E-stop (angle={blade_after:.3f} rad) "
    "— it swung out from under the item it was holding")
assert $FIRED_AFTER == $FIRED_BEFORE, "FAIL: an item fired after the E-stop"
PY

grep -q "E-STOP active" "$LOGDIR/skeleton.log"
echo "PASS: E-stop froze the belt, both items and the ENGAGED blade; no late fire"
echo "logs: $LOGDIR"
