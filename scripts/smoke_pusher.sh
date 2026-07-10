#!/usr/bin/env bash
# Smoke test: the pusher paddle shoves a box off the belt toward zone C (+Y).
# Isolates the mechanism (day 2 = nodes on their own): spawn the box right in
# front of the paddle at x=2.5, command /pusher/cmd, measure Y displacement.
# Run from repo root in WSL/Docker: bash scripts/smoke_pusher.sh
set -eu
cd "$(dirname "$0")/.."

export LIBGL_ALWAYS_SOFTWARE=1

ign gazebo -s -r -v 1 sim/worlds/cell.sdf > /tmp/gz_pusher.log 2>&1 &
GZ_PID=$!
trap 'kill $GZ_PID 2>/dev/null || true' EXIT
sleep 8

echo "--- spawning box in front of the paddle (x=2.5, y=0):"
ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$PWD/sim/models/items/box_300x200x200/model.sdf\", name: \"box1\", pose: {position: {x: 2.5, y: 0, z: 0.45}}"
sleep 2

box_y() {
    ign model -m box1 --pose | grep -A1 "XYZ" | tail -1 | tr -d "[]" | awk '{print $2}'
}

# JointController SUBSCRIBES to this topic (subscriptions are invisible to `ign topic -l`)
TOPIC=/pusher/cmd
echo "--- commanding pusher +1.2 m/s on $TOPIC"
ign topic -t "$TOPIC" -m ignition.msgs.Double -p "data: 1.2"

Y0=$(box_y)
sleep 3
Y1=$(box_y)
echo "--- box Y: $Y0 -> $Y1 (3s push)"
# Success = box left the belt on the +Y (zone C) side. Belt half-width is 0.25 m,
# so a box center past ~0.35 m has cleared the belt edge toward C. Precise delivery
# onto the C cage (y=1.2) is day-3 geometry (move the cage in / add a chute): the
# single paddle loses contact once the box drops off the 0.4 m-high belt edge.
python3 -c "
y1 = float('$Y1')
print(f'y1 = {y1:.2f} m (belt edge at y=0.25)')
assert y1 > 0.35, 'FAIL: box did not clear the belt toward zone C'
print('PASS: pusher pushes the box off the belt toward zone C')
"
