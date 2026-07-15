#!/usr/bin/env bash
# Stream robustness matrix: all 11 organizer items are exercised in a real
# multi-item stream at each selected seeded orientation.  Unlike run_matrix.sh,
# a world stays alive while 5-6 products share the belt, tracker and actuators.
#
# Usage:
#   bash scripts/run_stream_suite.sh [seed] [start_orient] [end_orient]
#   STREAM_SUITE_DRY_RUN=1 bash scripts/run_stream_suite.sh 0 0 2
#
# Each orientation is split into two episodes so the accepted feed schedule fits
# the finite belt stroke.  Same-zone products may run nose-to-tail; a zone change
# receives the planner's measured 3.1 m hold+retract clearance.  The Pen trails
# the slow Pouf by the separately measured 2.5 m transport floor.
set -u
set -o pipefail
cd "$(dirname "$0")/.."

SEED=${1:-0}
START_ORIENT=${2:-0}
END_ORIENT=${3:-2}
STREAM_RUNNER=${STREAM_RUNNER:-bash scripts/run_stream.sh}
EPISODE_TIMEOUT=${EPISODE_TIMEOUT:-300}
LOGDIR=${LOGDIR:-runs/stream_suite_$(date +%Y%m%d_%H%M%S)_seed${SEED}}

if ! [[ "$SEED" =~ ^[0-9]+$ ]]; then
    echo "ABORT: seed must be a non-negative integer, got '$SEED'" >&2
    exit 2
fi
if ! [[ "$START_ORIENT" =~ ^[0-9]+$ && "$END_ORIENT" =~ ^[0-9]+$ ]]; then
    echo "ABORT: orientation range must contain non-negative integers" >&2
    exit 2
fi
if ((START_ORIENT > END_ORIENT)); then
    echo "ABORT: orientation range is reversed: $START_ORIENT..$END_ORIENT" >&2
    exit 2
fi

MIXED_SPECS=(
    bottle:D:0
    plate:D:1.0
    box_400x400x300:C:3.1
    pouf:C:1.0
    pen:C:2.5
)
B_SPECS=(
    box_300x200x200:B:0
    lunchbox:B:1.0
    bag:B:1.0
    detergent:B:1.0
    cylinder:B:1.0
    helmet:B:1.0
)

total=0
passed=0
failed=0

run_episode() {
    local oi=$1
    local name=$2
    shift 2
    local specs=("$@")
    local episode="oi${oi}_${name}"
    local episode_log="$LOGDIR/$episode"
    total=$((total + 1))

    echo "[plan oi=$oi episode=$name items=${#specs[@]}] ${specs[*]}"
    if [ "${STREAM_SUITE_DRY_RUN:-0}" = 1 ]; then
        return 0
    fi

    set +e
    SEED="$SEED" ORIENT_INDEX="$oi" LOGDIR="$episode_log" \
        timeout --kill-after=15 "$EPISODE_TIMEOUT" $STREAM_RUNNER "${specs[@]}" \
        2>&1 | tee "$LOGDIR/${episode}.console.log"
    local rc=${PIPESTATUS[0]}
    set -e
    printf 'rc=%s\n' "$rc" > "$LOGDIR/${episode}.status"
    if [ -d "$episode_log" ]; then
        printf 'rc=%s\n' "$rc" > "$episode_log/runner.status"
    fi

    if [ "$rc" -eq 0 ]; then
        passed=$((passed + 1))
        echo "[episode $episode] PASS"
    else
        failed=$((failed + 1))
        if [ "$rc" -eq 124 ]; then
            echo "[episode $episode] TIMEOUT after ${EPISODE_TIMEOUT}s"
        else
            echo "[episode $episode] RUNNER_ERROR rc=$rc"
        fi
    fi
}

if [ "${STREAM_SUITE_DRY_RUN:-0}" != 1 ]; then
    mkdir -p "$LOGDIR"
fi

for oi in $(seq "$START_ORIENT" "$END_ORIENT"); do
    run_episode "$oi" mixed_cd "${MIXED_SPECS[@]}"
    run_episode "$oi" pass_b "${B_SPECS[@]}"
done

items_per_orientation=$((${#MIXED_SPECS[@]} + ${#B_SPECS[@]}))
orientations=$((END_ORIENT - START_ORIENT + 1))
if [ "${STREAM_SUITE_DRY_RUN:-0}" = 1 ]; then
    echo "=== stream suite dry-run seed=$SEED orientations=$START_ORIENT..$END_ORIENT: $total episodes, $((items_per_orientation * orientations)) planned items ==="
    exit 0
fi

echo "=== stream suite seed=$SEED orientations=$START_ORIENT..$END_ORIENT: episodes $passed/$total; failed $failed ==="
echo "planned items: $((items_per_orientation * orientations)); logs: $LOGDIR"
[ "$failed" -eq 0 ]
