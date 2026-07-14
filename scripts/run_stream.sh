#!/usr/bin/env bash
# Multi-item stream episode (day 10, week 2): several items fed onto the belt one
# after another and routed by the full loop, with no manual commands and no world
# restart between them. run_skeleton.sh proves ONE item end to end; this proves
# the contour is a CONVEYOR — items in the camera frame together, stable ids,
# independent controller timers, one action per item.
#
#   bash scripts/run_stream.sh [slug:zone:gap_m ...]
#   # default stream: box_400 -> C, pen 1 m behind it -> C, bottle 3.5 m back -> D
#
# Items are fed from ONE point at intervals in TIME (gap_m / belt speed), the way
# an infeed works — see scripts/stream_plan.py, which turns the specs into feed
# delays and REFUSES a stream the scene cannot carry: the belt stroke, and above
# all the diverter hold (a blade held across the belt sweeps whatever arrives
# during the hold, so a zone CHANGE needs ~3 m of air while two items bound for
# the SAME zone may ride nose to tail — that asymmetry is the throughput ceiling).
#
# Reproducibility: SEED and ORIENT_INDEX pick each spawn pose exactly as the
# matrix does (scripts/spawn_orientations.py) — ORIENT_INDEX=0 is the default STL
# pose, so the stream starts from the poses the census knows.
#
# Zone success criteria: scripts/zone_verdict.py (shared with run_skeleton.sh).
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

WORLD=${WORLD:-sim/worlds/cell_diverter.sdf}
ITEM_MODEL_ROOT=${ITEM_MODEL_ROOT:-sim/models/items}
SEED=${SEED:-0}
ORIENT_INDEX=${ORIENT_INDEX:-0}
PYTHON=${PYTHON:-python3}
# Diverter semantics on the pusher's topics (forwarded by skeleton.launch.py):
# the blade leads the item and holds while the belt slides it off the edge.
# stream_plan.py reads the SAME two numbers to size the gaps.
export HOLD_S=${HOLD_S:-2.5}
export FIRE_LEAD_S=${FIRE_LEAD_S:-0.5}
# The blade is POSITION-driven (rad): ENGAGE_CMD is the engaged angle (just inside
# the 0.95 limit — a velocity command pressed into that limit locked the joint and
# the blade never returned, which is what a stream, and only a stream, exposed),
# RETRACT_CMD is the parked angle. See src/controller_node.py.
export ENGAGE_CMD=${ENGAGE_CMD:-0.90}
export RETRACT_CMD=${RETRACT_CMD:-0.0}
# E-stop freezes the blade where it is instead of parking it (see run_skeleton.sh).
export ESTOP_HOLD_MECHANISM=${ESTOP_HOLD_MECHANISM:-true}
# Wall-clock poll budget: a stream outlives a single-item episode (the last item
# is still crossing the belt long after the first has landed).
POLL_ITERS=${RUN_STREAM_POLL_ITERS:-150}
LOGDIR=${LOGDIR:-runs/stream_$(date +%Y%m%d_%H%M%S)_seed${SEED}}

SPECS=("$@")
if [ ${#SPECS[@]} -eq 0 ]; then
    SPECS=(box_400x400x300:C:0 pen:C:1.0 bottle:D:3.5)
fi

# The plan aborts (nonzero) on a stream the scene cannot carry — do that BEFORE
# paying for a Gazebo boot, and print it so the log states what was attempted.
PLAN=$("$PYTHON" scripts/stream_plan.py "$WORLD" "${SPECS[@]}")
echo "=== stream plan (world=$WORLD seed=$SEED orient=$ORIENT_INDEX) ==="
echo "$PLAN"
[ "${STREAM_DRY_RUN:-0}" = 1 ] && exit 0

mkdir -p "$LOGDIR"

# Resolve every spawn pose BEFORE the episode: spawn_orientations.py loads the
# STL through trimesh (~1 s), which must not be paid inside the timed feed.
SLUGS=(); ZONES=(); DELAYS=(); SPAWN_X=(); SPAWN_Y=(); SPAWN_Z=(); QUATS=()
while read -r INDEX SLUG ZONE X DELAY; do
    QUAT=$("$PYTHON" scripts/spawn_orientations.py "$SEED" "$INDEX" "$ORIENT_INDEX" "$SLUG") || {
        echo "ABORT: spawn_orientations.py failed (seed=$SEED item=$INDEX)" >&2
        exit 1
    }
    # ...and the sideways offset that centres the item's BODY on the belt: the model
    # origin is not its centre, so spawning the origin at y=0 lands a rotated item
    # off the belt centre (see spawn_orientations.py).
    read -r OX OY OZ OW SZ SY <<< "$QUAT"
    if [ -z "$SY" ]; then
        echo "ABORT: orientation generator returned no spawn pose for $SLUG: '$QUAT'" >&2
        exit 1
    fi
    SLUGS+=("$SLUG"); ZONES+=("$ZONE"); DELAYS+=("$DELAY")
    SPAWN_X+=("$X"); SPAWN_Y+=("$SY"); SPAWN_Z+=("$SZ"); QUATS+=("$OX $OY $OZ $OW")
done <<< "$PLAN"

pkill -f "ign gazebo" 2>/dev/null || true
pkill -f "skeleton.launch" 2>/dev/null || true
pkill -f "src\..*_node" 2>/dev/null || true
pkill -f parameter_bridge 2>/dev/null || true
sleep 2

ign gazebo -s -r -v 0 "$WORLD" > "$LOGDIR/gazebo.log" 2>&1 &
sleep 10

# Pre-roll the belt against its lower joint limit: the slab's stroke is the
# episode's fuel and the whole stream has to fit inside it (see stream_plan.py).
ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: -3.0" > /dev/null
sleep 4
ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: 0" > /dev/null
sleep 1

ros2 launch launch/skeleton.launch.py > "$LOGDIR/skeleton.log" 2>&1 &
LAUNCH=$!

# Feed only once the belt is at full speed: the controller soft-starts it (a 0->1
# m/s step launches round items), and until the ramp is done a feed delay in
# seconds would not be the gap in metres the plan promised.
for _ in $(seq 1 60); do
    grep -q "soft-start done" "$LOGDIR/skeleton.log" && break
    sleep 0.5
done
if ! grep -q "soft-start done" "$LOGDIR/skeleton.log"; then
    echo "ABORT: belt never reached full speed — the controller did not come up" >&2
    kill $LAUNCH 2>/dev/null || true
    exit 1
fi
T0=$(date +%s.%N)

# Feed in the background so the poll loop can already time the arrivals of items
# fed earlier: with a 4.5 s stream the first item lands before the last is fed.
(
    ELAPSED=0
    for i in "${!SLUGS[@]}"; do
        WAIT=$("$PYTHON" -c "print(max(${DELAYS[$i]} - $ELAPSED, 0))")
        sleep "$WAIT"
        ELAPSED=${DELAYS[$i]}
        read -r OX OY OZ OW <<< "${QUATS[$i]}"
        ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
            --reptype ignition.msgs.Boolean --timeout 5000 \
            --req "sdf_filename: \"$PWD/$ITEM_MODEL_ROOT/${SLUGS[$i]}/model.sdf\", name: \"item$i\", pose: {position: {x: ${SPAWN_X[$i]}, y: ${SPAWN_Y[$i]}, z: ${SPAWN_Z[$i]}}, orientation: {x: $OX, y: $OY, z: $OZ, w: $OW}}" > /dev/null
        # announce the feed to the controller's watchdog (see run_skeleton.sh);
        # backgrounded so the CLI's startup does not skew the next feed delay
        ros2 topic pub -w 1 --once /infeed/fed std_msgs/msg/Empty > /dev/null 2>&1 &
        echo "fed item$i: ${SLUGS[$i]} -> ${ZONES[$i]} at t=${DELAYS[$i]}s"
    done
) &
FEEDER=$!

# Position AND orientation, from ONE query: the verdict scores the item's BODY, and the
# reported pose is only its ORIGIN — Gazebo rotates the model about the default pose's
# bottom, so at ORIENT_INDEX>0 the two drift apart by up to 349 mm (zone_verdict.py).
# At the default ORIENT_INDEX=0 the origin IS the contact point and the extra columns
# simply confirm it.
item_pose() {  # name -> "x y z roll pitch yaw"; an unfed item simply has no pose yet
    ign model -m "$1" --pose 2>/dev/null | grep -A2 "XYZ" | tail -2 \
        | tr -d "[]" | awk '{printf "%s %s %s ", $1, $2, $3}'
}

# One convex hull per item, dumped ONCE: the verdict scores the body, and doing that with
# trimesh inside the poll loop costs 1.33 s a call (see run_skeleton.sh). A stream polls
# every item on every lap, so the cost would be multiplied by the number of items.
declare -A HULL
for i in "${!SLUGS[@]}"; do
    HULL[$i]=/tmp/item_hull_${SLUGS[$i]}.txt
    "$PYTHON" scripts/body_pose.py --dump-hull "${SLUGS[$i]}" "${HULL[$i]}" 2>/dev/null \
        || HULL[$i]=/dev/null
done

declare -A ARRIVED_AT
declare -A LAST_POSE
LANDED=0
for _ in $(seq 1 "$POLL_ITERS"); do
    for i in "${!SLUGS[@]}"; do
        NAME="item$i"
        [ -n "${ARRIVED_AT[$NAME]:-}" ] && continue
        POSE=$(item_pose "$NAME")
        [ -z "$POSE" ] && continue          # not fed yet, or the CLI flaked — retry next lap
        read -r X Y Z RR PP YY <<< "$POSE"
        LAST_POSE[$NAME]="x=$X y=$Y z=$Z"
        if [ "$("$PYTHON" scripts/zone_verdict.py "${ZONES[$i]}" "$X" "$Y" "$Z" \
                "${HULL[$i]}" "$RR" "$PP" "$YY" 2>/dev/null)" = YES ]; then
            NOW=$(date +%s.%N)
            ARRIVED_AT[$NAME]=$("$PYTHON" -c "print(f'{$NOW - $T0:.1f}')")
            LANDED=$((LANDED + 1))
        fi
    done
    [ "$LANDED" = "${#SLUGS[@]}" ] && break
    sleep 0.5
done
wait $FEEDER 2>/dev/null || true

# The stream's own summary — arrivals (T0-relative, so Gazebo boot and the belt
# soft-start are excluded) and the takt — is echoed AND saved to the run dir, so
# scripts/measure_throughput.py can recover per-item landing times offline. The
# node stdout in skeleton.log carries the camera/decision/command stamps; this
# file carries the body-scored verdict arrivals (a different clock — see the
# parser). tee'd as one whole block, so no line is lost to a race on exit.
{
echo "=== stream result ==="
for i in "${!SLUGS[@]}"; do
    NAME="item$i"
    T=${ARRIVED_AT[$NAME]:-}
    if [ -n "$T" ]; then
        echo "$NAME ${SLUGS[$i]} -> ${ZONES[$i]}: PASS at t=${T}s (${LAST_POSE[$NAME]})"
    else
        echo "$NAME ${SLUGS[$i]} -> ${ZONES[$i]}: FAIL (${LAST_POSE[$NAME]:-no pose})"
    fi
done
echo "routed $LANDED/${#SLUGS[@]}"

# Throughput of the STREAM, not of a launch: the takt between items reaching
# their zones. Startup (Gazebo boot, node bring-up, belt ramp) is outside it by
# construction — t=0 is full belt speed, and the takt is between arrivals.
ARRIVALS=$(for i in "${!SLUGS[@]}"; do echo "${ARRIVED_AT[item$i]:-}"; done | grep -v '^$' | sort -n)
"$PYTHON" - <<PY
arrivals = [float(t) for t in """$ARRIVALS""".split()]
if len(arrivals) > 1:
    takt = (arrivals[-1] - arrivals[0]) / (len(arrivals) - 1)
    gaps = ", ".join(f"{b - a:.1f}" for a, b in zip(arrivals, arrivals[1:]))
    print(f"takt between arrivals: {takt:.1f} s ({gaps} s) => {60.0 / takt:.0f} items/min")
PY
} | tee "$LOGDIR/stream.log"

# Proof that perception kept the items apart: a merged blob would show ONE id, a
# jumping tracker MORE ids than items.
echo "=== ids seen by perception ==="
grep -oE "\[perception\]: item [0-9]+" "$LOGDIR/skeleton.log" | sort -u | tr '\n' ' ' || true
echo
echo "=== controller decisions ==="
grep -E "item [0-9]+: (B —|C —|D —)|FIRED|mis-sort|MISSED" "$LOGDIR/skeleton.log" \
    | sed -E 's/.*\[controller\]: //' | awk '!seen[$0]++' || true

kill $LAUNCH 2>/dev/null || true
sleep 1
pkill -f "skeleton.launch" 2>/dev/null || true
pkill -f "src\..*_node" 2>/dev/null || true
pkill -f parameter_bridge 2>/dev/null || true
pkill -f "ign gazebo" 2>/dev/null || true
echo "logs: $LOGDIR"
[ "$LANDED" = "${#SLUGS[@]}" ]
