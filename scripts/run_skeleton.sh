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
#
# errexit FIRST, before anything that can fail. It used to be armed only AFTER the
# two `source` lines below, so on a clean checkout the missing workspace did not stop
# the run: it carried on unsourced, found no `ign`, and died 100 lines later claiming
# "the belt never reached full speed" — a false diagnosis pointing at conveyor physics.
set -e
cd "$(dirname "$0")/.."
export LIBGL_ALWAYS_SOFTWARE=1
# install/ is a gitignored BUILD artifact, so a fresh clone has no setup.bash. Checked
# before sourcing ROS: this is the one failure we can name exactly, so name it loudly
# (same principle as zone_verdict.py) instead of leaving bash's "No such file".
ROS_INSTALL_ROOT=${ROS_INSTALL_ROOT:-install}
if [ ! -f "$ROS_INSTALL_ROOT/setup.bash" ]; then
    echo "ABORT: ROS workspace is not built ($ROS_INSTALL_ROOT/setup.bash is missing) — run:" >&2
    echo "    colcon build --packages-select ros_msgs" >&2
    exit 1
fi
source /opt/ros/humble/setup.bash
source "$ROS_INSTALL_ROOT/setup.bash"

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
# Spawn Y is not 0 either: the model origin is not the item's centre, so spawning
# the ORIGIN on the belt centre puts the BODY off it — a rotated helmet lands 154 mm
# to one side, inside the infeed rail, and the solver extrudes it upward for 40 s
# instead of the belt carrying it (the helmet's stable census failure). run_matrix.sh
# takes this per cell from spawn_orientations.spawn_offset_y_m.
SPAWN_Y=${SPAWN_Y:-0}
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
    # E-stop must FREEZE the blade, not send it home: on a positional mechanism a
    # 0.0 command is "park", which would swing the blade out from under the item
    # leaning on it — motion during an emergency stop.
    export ESTOP_HOLD_MECHANISM=${ESTOP_HOLD_MECHANISM:-true}
;; esac
# Gentleness metric seam (opt-in): dump the item's dynamic pose during the episode
# and print a peak speed/accel/impulse line. Off by default so the matrix and the
# pusher baseline are not slowed; compare_mechanisms.sh turns it on.
CAPTURE_DYNAMICS=${CAPTURE_DYNAMICS:-0}
# The matrix can point each cell at its own durable file. Keeping this configurable
# matters for rare physics failures: /tmp/dyn_trace.log is overwritten by the next
# episode, so the one failing trajectory used to disappear before it could be triaged.
DYNAMICS_TRACE=${DYNAMICS_TRACE:-/tmp/dyn_trace.log}
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

# record the item's dynamic pose for the whole episode (gentleness metric)
if [ "$CAPTURE_DYNAMICS" = 1 ]; then
    ign topic -e -t /world/cell/dynamic_pose/info > "$DYNAMICS_TRACE" 2>&1 &
    DYN_PID=$!
fi

T0=$(date +%s.%N)
ros2 launch launch/skeleton.launch.py > /tmp/skeleton_e2e.log 2>&1 &
LAUNCH=$!

# FEED ONTO A BELT ALREADY AT SPEED — do not start the belt UNDER the goods.
#
# This runner used to spawn the item first and then let the controller's soft-start ramp
# drag it up to 1 m/s. That is not how an infeed works, and it is what made the helmet
# "unstable": the helmet's convex hull stands on a 62x64 mm crown with its centre of mass
# 175 mm up, so it tips at 10 degrees, and the belt starting underneath tips it. Measured:
# with the old order it TUMBLES the whole way down (its own axis swings 0 -> 67 -> 125 deg)
# and its lateral position random-walks until, in 4 runs out of 19, it walks off the 0.5 m
# belt — the census's only remaining failure. Fed onto a moving belt instead, the same
# helmet rides dead centre (y = 0.011 m, three runs, identical) and never tumbles.
# run_stream.sh has always fed this way, which is why the stream never saw the defect.
# The item lands one clearance above a belt at 1 m/s, skids ~6 cm (mu=0.8) and is carried:
# the infeed rails hold it through that, exactly as they hold it in the stream.
# 30 s by default. This is a HARNESS constant, not a physical limit, and the
# comment above records that it has already produced one false diagnosis pointing
# at conveyor physics. It bites again on any configuration that boots slower than
# the shipped one: the three-head rig runs a ~50 s cycle against this world's ~30 s
# and overran the wait in 3 of 33 census cells. Overridable so a slow rig is
# measured rather than mis-attributed.
for _ in $(seq 1 "${SOFT_START_TRIES:-60}"); do
    grep -q "soft-start done" /tmp/skeleton_e2e.log && break
    sleep 0.5
done
if ! grep -q "soft-start done" /tmp/skeleton_e2e.log; then
    echo "ABORT: belt never reached full speed — the controller did not come up" >&2
    exit 1
fi
# WHAT THE "cycle" NUMBER IS MADE OF. It is measured from `ros2 launch`, so it
# carries the whole ROS stack coming up as well as the item's ride — and a rig
# comparison that quotes it alone cannot tell "our pipeline is slow" from "our
# harness starts more processes". The three-head rig runs a ~74-94 s median
# against the one-head world's ~23 s, and that gap has to be attributed before
# anything is optimised (a shipped line starts its stack ONCE; it pays bring-up
# per shift, not per parcel).
T_UP=$(date +%s.%N)

ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$PWD/$ITEM_MODEL_ROOT/$SLUG/model.sdf\", name: \"item\", pose: {position: {x: $SPAWN_X, y: $SPAWN_Y, z: $SPAWN_Z}, orientation: {x: $OX, y: $OY, z: $OZ, w: $OW}}" > /dev/null
# Announce the feed to the controller's watchdog: an announced item the camera
# never confirms latches the cell as a FEED JAM (the census feed_jam class was
# invisible before — the wedged item never yields a measurement). Backgrounded:
# `-w 1` waits for the subscription match, and a LATE announce only pushes the
# deadline later, never earlier.
ros2 topic pub -w 1 --once /infeed/fed std_msgs/msg/Empty > /dev/null 2>&1 &
sleep 2
T_FED=$(date +%s.%N)
# NO SIMULATOR-SPEED PROBE HERE, DELIBERATELY. An `ign topic -e ... -n 1` on the
# stats topic was tried and removed: it returned nothing in six episodes and
# still burned its full 5 s timeout in each, which on CI eats a twenty-fourth of
# the 120 s the whole contour smoke is allowed. Render load is separated from
# pipeline latency by the numbers already logged instead — the controller prints
# `cam->decision` (34 ms one head, 68 ms three), so a transit that grows by
# SECONDS while the decision grows by MILLISECONDS is the simulator rendering,
# not our code computing.

# Position AND orientation, from ONE query: the verdict scores the item's BODY, and
# the reported pose is only its ORIGIN (the default pose's bottom — Gazebo rotates the
# model about it, and for a turned bulky item the two are up to 349 mm apart). Two
# separate `ign model` calls would sample a moving item at two different instants and
# pair a pose with the wrong orientation.
item_pose() {  # x y z roll pitch yaw, with retries against ign CLI flakes
    local out=""
    for _ in 1 2 3; do
        out=$(ign model -m item --pose 2>/dev/null | grep -A2 "XYZ" | tail -2 \
              | tr -d "[]" | awk '{printf "%s %s %s ", $1, $2, $3}')
        [ -n "$out" ] && break
        sleep 0.5
    done
    echo "${out:-nan nan nan nan nan nan}"
}

# Reduce the item to its convex hull ONCE, here — not inside the poll loop. The verdict
# scores the item's BODY, which means rotating its mesh by the resting orientation; doing
# that with trimesh costs 1.33 s per call (numpy alone: 0.51 s) and the loop below runs up
# to 60 times, so the first body-scored census spent ~86 s extra per episode and reported
# the cells that overran the 180 s timeout as `physics_wedge` — the ruler's own weight,
# not physics. With the hull dumped, each poll is a few hundred rotations in plain Python.
# No hull (clean checkout, no STLs) => the verdict falls back to the reported pose, which
# is exact in the default pose that CI spawns.
HULL=/tmp/item_hull_$SLUG.txt
python3 scripts/body_pose.py --dump-hull "$SLUG" "$HULL" 2>/dev/null || HULL=/dev/null

# verdict polling is WALL-clock; under render load (record_skeleton_video.sh)
# RTF drops ~10x, so the recorder widens the window via this env override
POLL_ITERS=${RUN_SKELETON_POLL_ITERS:-60}

VERDICT=FAIL
# TRANSITION (day 11): the legacy verdict has to be LATCHED inside the loop, exactly as
# the old runner latched it — not evaluated once on the final pose. The two answer
# different questions otherwise: the old runner asked "did the item EVER satisfy the
# band", and an item can satisfy a band and then move on (bag oi=1 rode past the
# mechanisms on the belt, scoring B, and only afterwards drifted off the belt edge — its
# final pose is on the floor). Scoring the last pose would blame that on the ruler.
LEGACY=FAIL
for _ in $(seq 1 "$POLL_ITERS"); do
    read X Y Z RR PP YY <<< "$(item_pose)"
    if [ "$LEGACY" = FAIL ]; then
        OLD=$(python3 scripts/zone_verdict.py --legacy "$EXPECT" "$X" "$Y" "$Z" 2>/dev/null)
        [ "$OLD" = YES ] && LEGACY=PASS || true
    fi
    OK=$(python3 scripts/zone_verdict.py "$EXPECT" "$X" "$Y" "$Z" "$HULL" "$RR" "$PP" "$YY" 2>/dev/null)
    if [ "$OK" = YES ]; then
        VERDICT=PASS
        break
    fi
    sleep 0.5
done
T1=$(date +%s.%N)
CYCLE=$(python3 -c "print(f'{$T1 - $T0:.1f}')")

read X Y Z RR PP YY <<< "$(item_pose)"
echo "$SLUG -> $EXPECT: $VERDICT (pose x=$X y=$Y z=$Z, cycle ${CYCLE}s from launch)"
# The split behind that one number, so a rig comparison is attributable. ONE
# python start, not three: this line runs inside a 120 s CI budget.
python3 -c "print(f'  stages: bringup {$T_UP - $T0:.1f}s,'
                 f' feed {$T_FED - $T_UP:.1f}s,'
                 f' transit+verdict {$T1 - $T_FED:.1f}s')"
echo "  resting rpy: r=$RR p=$PP y=$YY"
python3 scripts/body_pose.py "$SLUG" "$X" "$Y" "$Z" "$RR" "$PP" "$YY" 2>/dev/null \
    | sed 's/^/  /' || true
# TRANSITION (day 11, remove after the re-census): $VERDICT scores the BODY, $LEGACY was
# latched in the loop above by the ORIGIN-scored rule every census up to #3 used. Printing
# both makes the cells the RULER moved visible per cell — as opposed to the cells physics
# moved, which show up as a cell differing between two runs (scripts/census_ruler_diff.py).
echo "  legacy origin-scored verdict: $LEGACY (body-scored: $VERDICT)"
grep -E "item [0-9]+:" /tmp/skeleton_e2e.log | tail -3 || true

if [ -n "$DYN_PID" ]; then
    kill "$DYN_PID" 2>/dev/null || true
    MASS=$(grep -m1 '<mass>' "$ITEM_MODEL_ROOT/$SLUG/model.sdf" \
           | sed -E 's/.*<mass>([0-9.]+)<.*/\1/')
    # --model-sdf adds the rotational bound on peak_accel: the trace is the model
    # ORIGIN, and for a tumbling item that point swings around the COM (lever arm
    # = the sdf's <inertial><pose>), so the bound separates "the goods were hit"
    # from "the ruler's point swung" (docs/decisions.md 2026-07-14).
    python3 scripts/capture_dynamics.py "$DYNAMICS_TRACE" --mass "${MASS:-1.0}" \
        --model-sdf "$ITEM_MODEL_ROOT/$SLUG/model.sdf" || true
fi

[ "$VERDICT" = PASS ]
