#!/usr/bin/env bash
# Smoke test: the swing-arm diverters guide a belt-carried item to its zone.
# Isolates the mechanism (no perception/controller): pre-engage a blade so it
# stands as a static angled wall, then run the belt forward so it carries the
# item into the blade, which slides it off the near edge onto the zone patch.
#   diverter_c (+Y blade) -> box   slides +Y onto zone C
#   diverter_d (-Y blade) -> plate slides -Y onto zone D
# The KEY contrast with smoke_pusher.sh: the item leaves the belt at belt speed
# (~1 m/s), guided — not launched at 2.5 m/s. Run from repo root in WSL/Docker:
#   bash scripts/smoke_diverter.sh
set -eu
cd "$(dirname "$0")/.."

export LIBGL_ALWAYS_SOFTWARE=1
WORLD=sim/worlds/cell_diverter.sdf

pkill -f "ign gazebo" 2>/dev/null || true
sleep 2

item_y() {
    ign model -m "$1" --pose | grep -A1 "XYZ" | tail -1 | tr -d "[]" | awk '{print $2}'
}

# one episode: fresh world, pre-engage $1's blade, spawn $2 upstream at $4,
# run the belt forward, print the item's final y after it is diverted.
episode() {
    local blade=$1 slug=$2 name=$3 spawn_x=$4
    ign gazebo -s -r -v 0 "$WORLD" > /tmp/gz_diverter.log 2>&1 &
    local GZ=$!
    sleep 10
    # engage the blade FIRST so it is a static angled wall when the item arrives
    # (a blade already in place guides; a blade swinging into a present item hits)
    ign topic -t "/$blade/cmd" -m ignition.msgs.Double -p "data: 2.5" > /dev/null
    sleep 1
    ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
        --reptype ignition.msgs.Boolean --timeout 5000 \
        --req "sdf_filename: \"$PWD/sim/models/items/$slug/model.sdf\", name: \"$name\", pose: {position: {x: $spawn_x, y: 0, z: 0.45}}" > /dev/null
    sleep 1
    # belt carries the item downstream into the engaged blade; 1.0 m/s from the
    # domain constant, forward long enough to reach the blade and be deflected
    ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: 1.0" > /dev/null
    sleep 3
    ign topic -t /conveyor/cmd_vel -m ignition.msgs.Double -p "data: 0" > /dev/null
    sleep 1
    item_y "$name"
    kill "$GZ" 2>/dev/null || true
    wait "$GZ" 2>/dev/null || true
    pkill -f "ign gazebo" 2>/dev/null || true
    sleep 2
}

# spawn well upstream of each pivot (C pivot x=2.75, D pivot x=3.25) so the
# moving belt carries the item into the blade face and slides it off the edge
echo "--- diverter_c: box -> zone C (+Y)"
YC=$(episode pusher_c box_300x200x200 box1 1.7)
echo "--- diverter_d: plate -> zone D (-Y)"
YD=$(episode pusher_d plate plate1 2.2)

# Success = item landed in its zone patch band (patches 0.8 m wide, |y|=0.9 ->
# y in [0.5, 1.3]), same criterion as smoke_pusher.sh
python3 -c "
yc, yd = float('$YC'), float('$YD')
print(f'box  y = {yc:.2f} m (zone C patch: 0.5..1.3)')
print(f'plate y = {yd:.2f} m (zone D patch: -1.3..-0.5)')
assert 0.5 <= yc <= 1.3, 'FAIL: box was not diverted onto the zone C patch'
assert -1.3 <= yd <= -0.5, 'FAIL: plate was not diverted onto the zone D patch'
print('PASS: both diverters guided items onto their zone patches')
"
