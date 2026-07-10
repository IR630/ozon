#!/usr/bin/env bash
# Smoke test: both pusher paddles shove items off the belt to their zones.
# Isolates the mechanisms: spawn an item right in front of a paddle at x=2.5,
# command the pusher topic, measure Y displacement.
#   pusher_c (-Y home) -> box  goes +Y toward zone C
#   pusher_d (+Y home) -> plate goes -Y toward zone D
# Run from repo root in WSL/Docker: bash scripts/smoke_pusher.sh
set -eu
cd "$(dirname "$0")/.."

export LIBGL_ALWAYS_SOFTWARE=1

pkill -f "ign gazebo" 2>/dev/null || true
sleep 2

item_y() {
    ign model -m "$1" --pose | grep -A1 "XYZ" | tail -1 | tr -d "[]" | awk '{print $2}'
}

# one episode: fresh world, spawn $2 in front of $1 (at its x), fire, print final y
episode() {
    local pusher=$1 slug=$2 name=$3 x=$4
    ign gazebo -s -r -v 0 sim/worlds/cell.sdf > /tmp/gz_pusher.log 2>&1 &
    local GZ=$!
    sleep 10
    ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
        --reptype ignition.msgs.Boolean --timeout 5000 \
        --req "sdf_filename: \"$PWD/sim/models/items/$slug/model.sdf\", name: \"$name\", pose: {position: {x: $x, y: 0, z: 0.45}}" > /dev/null
    sleep 2
    # positive command = fire, for BOTH pushers (pusher_d axis is -Y);
    # 2.5 m/s so the item leaves the belt edge with enough momentum to LAND
    # on the zone patch (the paddle cannot follow it off the 0.4 m drop)
    ign topic -t "/$pusher/cmd" -m ignition.msgs.Double -p "data: 2.5" > /dev/null
    sleep 3
    item_y "$name"
    kill "$GZ" 2>/dev/null || true
    wait "$GZ" 2>/dev/null || true
    pkill -f "ign gazebo" 2>/dev/null || true
    sleep 2
}

echo "--- pusher_c: box -> zone C (+Y)"
YC=$(episode pusher_c box_300x200x200 box1 2.5)
echo "--- pusher_d: plate -> zone D (-Y)"
YD=$(episode pusher_d plate plate1 3.0)

# Success = item landed in its zone patch band: patches are 0.8 m wide,
# centered at |y| = 0.9 -> y in [0.5, 1.3].
python3 -c "
yc, yd = float('$YC'), float('$YD')
print(f'box  y = {yc:.2f} m (zone C patch: 0.5..1.3)')
print(f'plate y = {yd:.2f} m (zone D patch: -1.3..-0.5)')
assert 0.5 <= yc <= 1.3, 'FAIL: box did not land on the zone C patch'
assert -1.3 <= yd <= -0.5, 'FAIL: plate did not land on the zone D patch'
print('PASS: both pushers land items on their zone patches')
"
