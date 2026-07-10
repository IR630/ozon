#!/usr/bin/env bash
# Smoke test: a box physically rides the conveyor belt.
# Headless world, spawn box_300x200x200 at the belt start, command belt
# surface speed (BELT_SPEED_M_S from src/constants.py), measure X displacement.
# Run from repo root in WSL/Docker: bash scripts/smoke_belt.sh
set -eu
cd "$(dirname "$0")/.."

# software rendering: deterministic headless run, survives WSL GL stack
export LIBGL_ALWAYS_SOFTWARE=1

ign gazebo -s -r -v 1 sim/worlds/cell.sdf > /tmp/gz_smoke.log 2>&1 &
GZ_PID=$!
trap 'kill $GZ_PID 2>/dev/null || true' EXIT
sleep 8

echo "--- spawning box:"
ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$PWD/sim/models/items/box_300x200x200/model.sdf\", name: \"box1\", pose: {position: {x: 0.3, y: 0, z: 0.45}}"
sleep 2

box_x() {
    ign model -m box1 --pose | grep -A1 "XYZ" | tail -1 | tr -d "[]" | awk '{print $1}'
}

SPEED=$(python3 -c "import sys; sys.path.insert(0, '.'); from src.constants import BELT_SPEED_M_S; print(BELT_SPEED_M_S)")
# JointController SUBSCRIBES to this topic (subscriptions are invisible to `ign topic -l`)
TOPIC=/conveyor/cmd_vel
echo "--- commanding belt speed $SPEED on $TOPIC"
ign topic -t "$TOPIC" -m ignition.msgs.Double -p "data: $SPEED"

X0=$(box_x)
sleep 2
X1=$(box_x)
echo "--- box X: $X0 -> $X1 (2s at $SPEED m/s)"
python3 -c "
dx = float('$X1') - float('$X0')
print(f'dx = {dx:.2f} m')
assert dx > 0.5, 'FAIL: box did not ride the belt'
print('PASS: box rides the belt')
"
