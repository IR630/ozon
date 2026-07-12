#!/usr/bin/env bash
# Mechanism comparison (day 5, P1): the SAME item flow through each candidate
# mechanism, one summary table for the day-6 decision. Each episode is one
# run_skeleton.sh pass (fresh Gazebo, full contour, no manual commands) with
# CAPTURE_DYNAMICS=1, so every cell yields four numbers on identical terms:
#   verdict      — routing success (run_skeleton zone criteria)
#   cycle_s      — episode wall-clock from launch to verdict
#   peak_accel   — gentleness (m/s^2, scripts/capture_dynamics.py)
#   peak_impulse — gentleness (N*s)
#
# Run from repo root in WSL/Docker:
#   bash scripts/compare_mechanisms.sh
# Item flow is overridable (slug:zone pairs), e.g. to add boundary items:
#   ITEMS="box_400x400x300:C plate:D bottle:D" bash scripts/compare_mechanisms.sh
#
# The mechanism seam is run_skeleton.sh's WORLD env (command topics identical,
# nodes unchanged); the diverter's blade-hold time rides the same seam (HOLD_S).
set -u
cd "$(dirname "$0")/.."

# B item included on purpose: it exercises the no-intervention path on both
# worlds (a mechanism must not disturb a passing B item) and gives the belt-only
# dynamics baseline the C/D numbers are read against.
ITEMS=${ITEMS:-"box_300x200x200:B box_400x400x300:C plate:D"}
CELL_TIMEOUT=${CELL_TIMEOUT:-180}
SKELETON=${SKELETON:-bash scripts/run_skeleton.sh}
declare -A WORLDS=(
    [pusher]=sim/worlds/cell.sdf
    [diverter]=sim/worlds/cell_diverter.sdf
)

summary=()
for mech in pusher diverter; do
    world=${WORLDS[$mech]}
    echo "=== $mech ($world) ==="
    for pair in $ITEMS; do
        slug=${pair%%:*}
        zone=${pair##*:}
        log="/tmp/compare_${mech}_${slug}.log"
        rc=0
        WORLD=$world CAPTURE_DYNAMICS=1 \
            timeout --kill-after=15 "$CELL_TIMEOUT" $SKELETON "$slug" "$zone" \
            > "$log" 2>&1 || rc=$?
        if [ "$rc" = 0 ]; then
            v=PASS
        elif [ "$rc" = 124 ] || [ "$rc" = 137 ]; then
            v=TIMEOUT
        else
            v=FAIL
        fi
        cycle=$(grep -oE "cycle [0-9.]+s" "$log" | tail -1 | grep -oE "[0-9.]+")
        gent=$(grep -m1 "gentleness:" "$log" || true)
        accel=$(grep -oE "peak_accel=[0-9.]+" <<< "$gent" | grep -oE "[0-9.]+")
        impulse=$(grep -oE "peak_impulse=[0-9.]+" <<< "$gent" | grep -oE "[0-9.]+")
        row=$(printf "%-9s %-16s -> %s: %-7s cycle=%-6s peak_accel=%-8s peak_impulse=%s" \
              "$mech" "$slug" "$zone" "$v" "${cycle:-?}s" "${accel:-?}" "${impulse:-?}")
        echo "$row"
        summary+=("$row")
    done
done

echo
echo "=== mechanism comparison (identical flow: $ITEMS) ==="
printf '%s\n' "${summary[@]}"
