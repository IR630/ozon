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

SPEED=$(python3 -c "import sys; sys.path.insert(0,'.'); from src.constants import BELT_SPEED_M_S; print(BELT_SPEED_M_S)")
fails=0

item_pose() {  # echo "x z" of the spawned model origin (item bottom rests at belt top ~0.4)
    ign model -m item --pose | grep -A1 "XYZ" | tail -1 | tr -d "[]" | awk '{print $1, $3}'
}

for dir in sim/models/items/*/; do
    slug=$(basename "$dir")
    ign gazebo -s -r -v 0 sim/worlds/cell.sdf > /tmp/gz_stab.log 2>&1 &
    GZ=$!
    sleep 8
    ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
        --reptype ignition.msgs.Boolean --timeout 5000 \
        --req "sdf_filename: \"$PWD/${dir}model.sdf\", name: \"item\", pose: {position: {x: 0.0, y: 0, z: 0.5}}" > /dev/null
    sleep 2
    read rx0 rz0 <<< "$(item_pose)"                       # after settling on the static belt
    ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: $SPEED" > /dev/null
    sleep 2
    read rx1 rz1 <<< "$(item_pose)"                       # after riding the belt
    kill "$GZ" 2>/dev/null || true
    wait "$GZ" 2>/dev/null || true
    sleep 1

    verdict=$(python3 -c "
x0, z0 = $rx0, $rz0
x1, z1 = $rx1, $rz1
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
