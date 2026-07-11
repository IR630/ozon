#!/usr/bin/env bash
# Routing matrix (day 4, P1+P2): all 11 items x N seeded orientations through the
# full contour, aggregated into a routing-correctness score. Each cell is one
# run_skeleton.sh episode (fresh Gazebo, world restart) with a seeded spawn
# orientation from scripts/spawn_orientations.py — reproducible from the seed.
#
# Run from repo root in WSL/Docker:
#   bash scripts/run_matrix.sh [seed] [N]        # default seed=0, N=3
#
# One cell ~40 s (Gazebo boot + episode); 11 x N cells run sequentially.
# orient_index 0 is the default STL pose, so the matrix includes the day-3 runs.
set -u
cd "$(dirname "$0")/.."

SEED=${1:-0}
N=${2:-3}

# slug -> expected zone, from the reference table docs/md/models.md.
SLUGS=(bottle box_300x200x200 box_400x400x300 lunchbox bag detergent pouf pen plate cylinder helmet)
ZONES=(D      B               C               B        B   B         C    C   D     B        B)

pass=0
total=0
declare -A item_pass
declare -A item_total

for i in "${!SLUGS[@]}"; do
    slug=${SLUGS[$i]}
    zone=${ZONES[$i]}
    item_total[$slug]=0
    item_pass[$slug]=0
    for oi in $(seq 0 $((N - 1))); do
        read OX OY OZ OW <<< "$(python3 scripts/spawn_orientations.py "$SEED" "$i" "$oi")"
        log="/tmp/matrix_${slug}_${oi}.log"
        total=$((total + 1))
        item_total[$slug]=$((item_total[$slug] + 1))
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

echo "=== matrix seed=$SEED N=$N: routing correctness $pass/$total ==="
for i in "${!SLUGS[@]}"; do
    slug=${SLUGS[$i]}
    echo "  $slug -> ${ZONES[$i]}: ${item_pass[$slug]}/${item_total[$slug]}"
done
[ "$pass" = "$total" ]
