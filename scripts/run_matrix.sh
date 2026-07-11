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
set -u
cd "$(dirname "$0")/.."

SEED=${1:-0}
N=${2:-3}
PYTHON=${PYTHON:-python3}
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
        quat=$("$PYTHON" scripts/spawn_orientations.py "$SEED" "$i" "$oi") || {
            echo "ABORT: spawn_orientations.py failed (seed=$SEED item=$i oi=$oi)" >&2
            exit 1
        }
        read OX OY OZ OW _ <<< "$quat"
        if [ -z "$OW" ]; then
            echo "ABORT: orientation generator returned <4 values (seed=$SEED item=$i oi=$oi): '$quat'" >&2
            exit 1
        fi
        log="/tmp/matrix_${slug}_${oi}.log"
        total=$((total + 1))
        item_total[$slug]=$((item_total[$slug] + 1))
        if [ "${MATRIX_DRY_RUN:-0}" = 1 ]; then
            echo "[plan $slug oi=$oi -> $zone] quat=$OX $OY $OZ $OW"
            continue
        fi
        if ORIENT_X=$OX ORIENT_Y=$OY ORIENT_Z=$OZ ORIENT_W=$OW \
           bash scripts/run_skeleton.sh "$slug" "$zone" > "$log" 2>&1; then
            v=PASS
            pass=$((pass + 1))
            item_pass[$slug]=$((item_pass[$slug] + 1))
        else
            v=FAIL
        fi
        # perception line "item N: WxHxD mm K=.. at (..)" — measured dims vs models.md
        meas=$(grep -E "item [0-9]+: [0-9]" "$log" | tail -1)
        echo "[$slug oi=$oi -> $zone] $v | ${meas:-no measurement in log}"
    done
done

if [ "${MATRIX_DRY_RUN:-0}" = 1 ]; then
    echo "=== matrix dry-run seed=$SEED N=$N items=$START_ITEM..$END_ITEM: $total cells ==="
    exit 0
fi

echo "=== matrix seed=$SEED N=$N: routing correctness $pass/$total ==="
for i in $(seq "$START_ITEM" "$END_ITEM"); do
    slug=${SLUGS[$i]}
    echo "  $slug -> ${ZONES[$i]}: ${item_pass[$slug]}/${item_total[$slug]}"
done
[ "$pass" = "$total" ]
