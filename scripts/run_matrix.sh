#!/usr/bin/env bash
# Routing matrix (day 4, P1+P2): all 11 items x N seeded orientations through the
# full contour, aggregated into a routing-correctness score. Each cell is one
# run_skeleton.sh episode (fresh Gazebo, world restart) with a seeded spawn
# orientation from scripts/spawn_orientations.py — reproducible from the seed.
#
# Run from repo root in WSL/Docker:
#   bash scripts/run_matrix.sh [seed] [N] [start_item] [end_item]
#   # defaults: seed=0, N=3, item indices 0..10
#
# Resume an interrupted tail without repeating completed multi-minute cells:
#   bash scripts/run_matrix.sh 0 3 7 10
# Preview the exact selected cells/quaternions without starting Gazebo:
#   MATRIX_DRY_RUN=1 bash scripts/run_matrix.sh 0 3 7 10
#
# One cell ~40 s (Gazebo boot + episode); 11 x N cells run sequentially.
# orient_index 0 is the default STL pose, so the matrix includes the day-3 runs.
#
# Cap each cell's wall-clock — a wedged Gazebo episode (a foreground `ign` query
# in run_skeleton.sh blocks forever when physics blows up, e.g. pouf) would else
# hang the whole matrix. A capped cell is recorded TIMEOUT, and the loop moves on.
#   CELL_TIMEOUT=180 bash scripts/run_matrix.sh 0 3
set -u
cd "$(dirname "$0")/.."

SEED=${1:-0}
N=${2:-3}
PYTHON=${PYTHON:-python3}
# Episode logs live in the repo, NOT in /tmp: a WSL distro shuts down when its
# last process exits and wipes /tmp on the next boot — the seed-0 diverter census
# lost all 33 episode logs that way, minutes after finishing. runs/ is gitignored.
LOGDIR=${LOGDIR:-runs/matrix_$(date +%Y%m%d_%H%M%S)_seed${SEED}}
# Per-cell wall-clock ceiling (s) and the runner it caps. SKELETON is overridable
# so tests can inject a stub, exactly like the PYTHON seam above.
# RAISED 180 -> 400 on 30.07 with the shipped rig. 180 s was sized for the one-head
# world (~23 s cycle); the two-head rig runs ~43.9 s and lost cells to rc=124 at 180
# on a loaded machine (bottle oi=1 — see docs/cameras-under-noise-brief.md). A capped
# cell records TIMEOUT, which scores as a routing miss the rig never actually made,
# so the cap must sit far past the cycle, not near it. Pinned by tests/test_run_matrix.py.
CELL_TIMEOUT=${CELL_TIMEOUT:-400}
SKELETON=${SKELETON:-bash scripts/run_skeleton.sh}
# Opt-in full pose trace per cell. A normal census stays lightweight; a targeted
# flaky-row replay can set SAVE_DYNAMICS=1 and retain the trajectory that failed
# instead of losing the shared /tmp trace when the next episode starts.
SAVE_DYNAMICS=${SAVE_DYNAMICS:-${CAPTURE_DYNAMICS:-0}}
# THE MECHANISM THE CENSUS MEASURES. run_skeleton.sh defaults to the ballistic pusher
# (cell.sdf) because that is the day-3 baseline and what CI's e2e drives — but the census
# is the milestone metric, and the milestone's mechanism is the DIVERTER. Leaving it
# implicit cost a full day: three censuses were run without WORLD set, silently landed on
# the pusher, and produced 30-31/33 for the abandoned mechanism. The pusher's belt has
# only 6.4 m of stroke against the diverter's 20 m (the day-10 fix was applied to the
# diverter world alone), so it runs out mid-episode and B items simply STOP on the belt
# around x=3.4-4.9 — which the old x>=3.5 band happily scored as "rode to the end".
# Name the world here, and print it, so a census can never again be silent about what it
# measured. Override explicitly (compare_mechanisms.sh does) to measure the baseline.
# Production is the TWO-head rig (30.07). Other rigs stay one variable away:
# WORLD=sim/worlds/cell_diverter.sdf BRIDGE_CONFIG=bridge.yaml for one head,
# cell_diverter_3cam.sdf + bridge_3cam.yaml for three.
export WORLD=${WORLD:-sim/worlds/cell_diverter_2cam.sdf}
SLUGS=(bottle box_300x200x200 box_400x400x300 lunchbox bag detergent pouf pen plate cylinder helmet)
ZONES=(D      B               C               B        B   B         C    C   D     B        B)
START_ITEM=${3:-0}
END_ITEM=${4:-$((${#SLUGS[@]} - 1))}

if ! [[ "$N" =~ ^[1-9][0-9]*$ ]]; then
    echo "ABORT: N must be a positive integer, got '$N'" >&2
    exit 2
fi
if ! [[ "$START_ITEM" =~ ^[0-9]+$ && "$END_ITEM" =~ ^[0-9]+$ ]]; then
    echo "ABORT: item range must contain integer indices, got '$START_ITEM..$END_ITEM'" >&2
    exit 2
fi
if ((START_ITEM > END_ITEM || END_ITEM >= ${#SLUGS[@]})); then
    echo "ABORT: item range must fit 0..$((${#SLUGS[@]} - 1)), got '$START_ITEM..$END_ITEM'" >&2
    exit 2
fi

# State the mechanism up front, in the log a report would read (IMaSKoT hit the same
# trap independently and added this line; the default above is the other half of it —
# a WARNING on stderr is easy to miss in a 35-minute census log, so the SAFE default
# matters more than the notice).
echo "world: $WORLD"

# Only now, past argument validation and only for a real run: an ABORT or a
# dry-run must not leave an empty log directory behind.
[ "${MATRIX_DRY_RUN:-0}" = 1 ] || mkdir -p "$LOGDIR"

pass=0
total=0
declare -A item_pass
declare -A item_total

for i in $(seq "$START_ITEM" "$END_ITEM"); do
    slug=${SLUGS[$i]}
    zone=${ZONES[$i]}
    item_total[$slug]=0
    item_pass[$slug]=0
    for oi in $(seq 0 $((N - 1))); do
        # Fail LOUD if the seeded generator breaks: an empty read would leave the
        # ORIENT_* vars empty and run_skeleton.sh would silently fall back to the
        # identity pose (${ORIENT_X:-0}...), so every oi>0 cell would re-run the
        # default pose while still scoring PASS/FAIL — a silent, wrong matrix.
        # The slug makes the generator also return the spawn HEIGHT for this pose:
        # a fixed z buries tall/turned items inside the belt and the solver ejects
        # them at the spawn (the census feed_jams — see spawn_orientations.py).
        quat=$("$PYTHON" scripts/spawn_orientations.py "$SEED" "$i" "$oi" "$slug") || {
            echo "ABORT: spawn_orientations.py failed (seed=$SEED item=$i oi=$oi)" >&2
            exit 1
        }
        read OX OY OZ OW SZ SY <<< "$quat"
        if [ -z "$OW" ]; then
            echo "ABORT: orientation generator returned <4 values (seed=$SEED item=$i oi=$oi): '$quat'" >&2
            exit 1
        fi
        log="$LOGDIR/matrix_${slug}_${oi}.log"
        # Deliberately NOT matrix_*.log: triage_matrix.py and census_ruler_diff.py
        # reserve that glob for episode summaries and would parse a raw pose trace
        # as an extra census cell.
        dynamics_log="$LOGDIR/dynamic_pose_${slug}_${oi}.log"
        total=$((total + 1))
        item_total[$slug]=$((item_total[$slug] + 1))
        if [ "${MATRIX_DRY_RUN:-0}" = 1 ]; then
            echo "[plan $slug oi=$oi -> $zone] quat=$OX $OY $OZ $OW"
            continue
        fi
        rc=0
        # CELL_OI names the pose inside the item's row. run_skeleton.sh ignores it;
        # the video recorder (scripts/record_census_cell.sh) needs it, because the
        # three orientations of one item would otherwise write one filename three
        # times and the reel would show pose 3 of every item, scored 33/33.
        ORIENT_X=$OX ORIENT_Y=$OY ORIENT_Z=$OZ ORIENT_W=$OW CELL_OI=$oi \
            SPAWN_Z=${SZ:-0.5} SPAWN_Y=${SY:-0} \
            CAPTURE_DYNAMICS=$SAVE_DYNAMICS DYNAMICS_TRACE="$dynamics_log" \
            timeout --kill-after=15 "$CELL_TIMEOUT" $SKELETON "$slug" "$zone" \
            > "$log" 2>&1 || rc=$?
        status="$LOGDIR/matrix_${slug}_${oi}.status"
        status_tmp="${status}.tmp.$$"
        if ! printf 'rc=%s\n' "$rc" > "$status_tmp" || ! mv "$status_tmp" "$status"; then
            echo "ABORT: cannot save runner status for $slug oi=$oi" >&2
            exit 1
        fi

        terminal=$(grep -E "^${slug} -> ${zone}: (PASS|FAIL) " "$log" | tail -1 || true)
        terminal_verdict=""
        case "$terminal" in
            "${slug} -> ${zone}: PASS "*) terminal_verdict=PASS ;;
            "${slug} -> ${zone}: FAIL "*) terminal_verdict=FAIL ;;
        esac

        if [ "$rc" = 0 ] && [ "$terminal_verdict" = PASS ]; then
            v=PASS
            pass=$((pass + 1))
            item_pass[$slug]=$((item_pass[$slug] + 1))
        elif [ "$rc" = 1 ] && [ "$terminal_verdict" = FAIL ]; then
            v=FAIL
        elif [ "$rc" = 124 ] && [ -z "$terminal_verdict" ]; then
            v=TIMEOUT
        elif [ "$rc" != 0 ] && [ -z "$terminal_verdict" ]; then
            v=RUNNER_ERROR
        else
            v=HARNESS_ERROR
        fi
        # perception line "item N: WxHxD mm K=.. at (..)" — measured dims vs models.md
        meas=$(grep -E "item [0-9]+: [0-9]" "$log" | tail -1)
        echo "[$slug oi=$oi -> $zone] $v | ${meas:-no measurement in log}"
    done
done

if [ "${MATRIX_DRY_RUN:-0}" = 1 ]; then
    echo "=== matrix dry-run world=$WORLD seed=$SEED N=$N items=$START_ITEM..$END_ITEM: $total cells ==="
    exit 0
fi

{
    echo "=== matrix world=$WORLD seed=$SEED N=$N: routing correctness $pass/$total ==="
    for i in $(seq "$START_ITEM" "$END_ITEM"); do
        slug=${SLUGS[$i]}
        echo "  $slug -> ${ZONES[$i]}: ${item_pass[$slug]}/${item_total[$slug]}"
    done
} | tee "$LOGDIR/summary.log"
echo "logs: $LOGDIR (triage: python3 scripts/triage_matrix.py --logdir $LOGDIR)"
[ "$pass" = "$total" ]
