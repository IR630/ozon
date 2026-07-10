#!/usr/bin/env bash
# Stability sweep (day 2, P5): each of the 11 items must rest on the belt
# (no fall-through, no blow-up) and ride with it. The world is restarted per
# item because the prismatic belt has a limited ±3.2 m travel per episode.
# Run from repo root in WSL/Docker: bash scripts/check_stability.sh
set -u
cd "$(dirname "$0")/.."
export LIBGL_ALWAYS_SOFTWARE=1

if [ ! -d sim/models/items/bottle ]; then
    python3 scripts/build_item_models.py
fi

# orphaned servers from earlier/killed runs answer /world/cell services too and
# make `ign model` time out — start from a clean slate
pkill -f "ign gazebo" 2>/dev/null
sleep 2

SPEED=$(python3 -c "import sys; sys.path.insert(0,'.'); from src.constants import BELT_SPEED_M_S; print(BELT_SPEED_M_S)")
fails=0

# Soft-start: step the belt up to SPEED instead of commanding it instantly.
# An instant 0->1 m/s jerk launches round items (helmet bounced 0.32 m up and
# flew off; bottle rolled sideways off the edge) — diagnosed via trajectory
# logs, docs/experiments.md. Real conveyors ramp too; the day-3 controller
# must do the same (docs/decisions.md).
belt_soft_start() {
    for frac in 0.125 0.25 0.375 0.5 0.625 0.75 0.875 1.0; do
        v=$(python3 -c "print($SPEED * $frac)")
        ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: $v" > /dev/null
        sleep 0.15   # each `ign topic` call itself adds ~0.5 s
    done
}

item_pose() {  # echo "x z" of the spawned model origin (item bottom rests at belt top ~0.4)
    # `ign model` flakes with "Service call timed out" now and then -> retry
    local out=""
    for _ in 1 2 3 4 5; do
        out=$(ign model -m item --pose 2>/dev/null | grep -A1 "XYZ" | tail -1 | tr -d "[]" | awk '{print $1, $3}')
        [ -n "$out" ] && break
        sleep 1
    done
    echo "${out:-nan nan}"
}

for dir in sim/models/items/*/; do
    slug=$(basename "$dir")
    ign gazebo -s -r -v 0 sim/worlds/cell.sdf > /tmp/gz_stab.log 2>&1 &
    GZ=$!
    sleep 10   # the very first boot is the slowest; a short sleep made the first pose query time out
    # pre-roll the belt to its -3.2 m joint limit: doubles the usable travel to
    # 6.4 m so the belt never slams its limit inside the measurement window
    # (the limit-stop jerk threw the pouf off in an earlier run)
    ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: -1.0" > /dev/null
    sleep 4
    ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: 0" > /dev/null
    sleep 1
    ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
        --reptype ignition.msgs.Boolean --timeout 5000 \
        --req "sdf_filename: \"$PWD/${dir}model.sdf\", name: \"item\", pose: {position: {x: 0.0, y: 0, z: 0.5}}" > /dev/null
    sleep 2
    read rx0 rz0 <<< "$(item_pose)"                       # after settling on the static belt
    belt_soft_start                                       # ~0.45 m of travel during the ramp
    sleep 2                                               # + 2 s at full speed (2.5 m total < 3.2 m belt travel)
    read rx1 rz1 <<< "$(item_pose)"                       # after riding the belt
    kill "$GZ" 2>/dev/null || true
    wait "$GZ" 2>/dev/null || true
    pkill -f "ign gazebo" 2>/dev/null    # `ign gazebo` forks; the child survives kill $GZ
    sleep 2

    verdict=$(python3 -c "
x0, z0 = float('$rx0'), float('$rz0')    # 'nan' when the pose query kept timing out
x1, z1 = float('$rx1'), float('$rz1')
rest_ok = 0.35 <= z0 <= 1.0          # sits on the belt, not sunk through / blown up
end_ok  = 0.35 <= z1 <= 1.0
rides   = (x1 - x0) > 1.0            # carried along +x with the belt (~1 m/s * 2 s)
ok = rest_ok and end_ok and rides
print('PASS' if ok else 'FAIL', f'z_rest={z0:.2f} z_end={z1:.2f} dx={x1-x0:.2f}')
")
    echo "$slug: $verdict"
    [[ $verdict == PASS* ]] || fails=$((fails + 1))
done

echo "--- unstable: $fails/11"
exit "$fails"
