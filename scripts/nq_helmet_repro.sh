#!/usr/bin/env bash
# Night-queue reproduction of the two multi-seed routing misses, to CAPTURE the
# per-frame K distribution the aggregated logs never kept.
#
#   bash scripts/nq_helmet_repro.sh [R]   # default R=6 repeats per cell
#
# Why this exists (docs/experiments.md + docs/decisions.md, 22-23.07): the
# multi-seed census routed 163/165. The two misses were BOTH classification, both
# printing an aggregated k=0.800 that routed D under the strict `k > 0.8` rule.
# The morning-of-23.07 diagnosis ("the flatness clamp pins K exactly on the
# threshold, the contour decides on the median, hence D") was SELF-REFUTED the
# same afternoon: an offline noise probe over the 15 clamped poses put ZERO of
# them in D. So the real reason the CONTOUR said D on `helmet oi=1 seed 1` is
# still unowned — offline single-frame noise does not reproduce it, and we hold
# no per-FRAME K series behind the aggregate median.
#
# This runner replays the exact two miss cells R times with a per-cell NODE_LOG
# preserved under runs/nq_helmet_repro/, so BOTH the perception per-frame lines
# (`... mm K=0.XXXX ...`, 4 decimals since 6cb5372) and the classifier aggregate
# (`item N: <B|C|D> (k=0.XXXXXX, ...)`, 6 decimals) survive per repeat. The pose
# is the seeded one from spawn_orientations.py — same source run_matrix.sh uses —
# so this is the census cell, not an approximation of it.
#
# Aggregate the captured distribution afterwards with:
#   python3 scripts/nq_helmet_summary.py runs/nq_helmet_repro
set -e
cd "$(dirname "$0")/.."
R=${1:-6}
PYTHON=${PYTHON:-python3}
OUT=runs/nq_helmet_repro
mkdir -p "$OUT"

# The two census misses, verbatim: "slug item_index seed oi expected_zone".
#   helmet -> B (i=10) missed on seed 1, oi 1 (docs/experiments.md 22.07)
#   bag    -> B (i=4)  missed on seed 4, oi 2 (same)
CELLS=(
    "helmet 10 1 1 B"
    "bag 4 4 2 B"
)

summary="$OUT/summary.log"
: > "$summary"
echo "=== nq_helmet_repro R=$R $(date -u +%Y-%m-%dT%H:%M:%SZ) ===" | tee -a "$summary"

for cell in "${CELLS[@]}"; do
    read -r slug i seed oi zone <<< "$cell"
    quat=$("$PYTHON" scripts/spawn_orientations.py "$seed" "$i" "$oi" "$slug") || {
        echo "ABORT: spawn_orientations.py failed ($slug seed=$seed oi=$oi)" >&2
        exit 1
    }
    read -r OX OY OZ OW SZ SY <<< "$quat"
    if [ -z "$OW" ]; then
        echo "ABORT: orientation generator returned <4 values ($slug seed=$seed oi=$oi): '$quat'" >&2
        exit 1
    fi
    for r in $(seq 1 "$R"); do
        node_log="$OUT/${slug}_seed${seed}_oi${oi}_rep${r}.node.log"
        run_log="$OUT/${slug}_seed${seed}_oi${oi}_rep${r}.run.log"
        rc=0
        ORIENT_X=$OX ORIENT_Y=$OY ORIENT_Z=$OZ ORIENT_W=$OW \
            SPAWN_Z=${SZ:-0.5} SPAWN_Y=${SY:-0} \
            NODE_LOG="$node_log" \
            timeout --kill-after=15 180 bash scripts/run_skeleton.sh "$slug" "$zone" \
            > "$run_log" 2>&1 || rc=$?
        # Final aggregated verdict (last running-median line) + how many per-frame
        # measurements fed it. A miss shows category D with an n-frame median at 0.8.
        agg=$(grep -E "item [0-9]+: [BCD] \(k=" "$node_log" 2>/dev/null | tail -1 || true)
        nframes=$(grep -cE "item [0-9]+: [0-9].* mm K=" "$node_log" 2>/dev/null || echo 0)
        printf '[%s seed%s oi%s rep%s] rc=%s frames=%s | %s\n' \
            "$slug" "$seed" "$oi" "$r" "$rc" "$nframes" "${agg:-NO-VERDICT}" | tee -a "$summary"
    done
done
echo "=== done -> $OUT ===" | tee -a "$summary"
