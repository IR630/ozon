#!/usr/bin/env bash
# Boot Gazebo + the ros_gz bridge, spawn ONE item at rest UNDER the camera
# (x=1.5, belt stationary — no controller), dump depth/RGB PNGs. For offline
# perception debugging (day 4: vertical-section K on real depth frames).
#
#   bash scripts/dump_item_frame.sh <slug> [outdir]
#   ORIENT_{X,Y,Z,W}=... bash scripts/dump_item_frame.sh <slug>   # seeded pose
cd "$(dirname "$0")/.."
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
source install/setup.bash
set -e

SLUG=${1:?usage: dump_item_frame.sh <slug> [outdir]}
OUT=${2:-/tmp/frames_$SLUG}
OX=${ORIENT_X:-0}; OY=${ORIENT_Y:-0}; OZ=${ORIENT_Z:-0}; OW=${ORIENT_W:-1}

pkill -f "ign gazebo" 2>/dev/null || true
pkill -f parameter_bridge 2>/dev/null || true
sleep 2

ign gazebo -s -r -v 0 sim/worlds/cell.sdf > /tmp/gz_dump.log 2>&1 &
sleep 10

# spawn directly under the camera and let it settle to its rest pose; belt is
# never commanded, so the item stays in the camera window
ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$PWD/sim/models/items/$SLUG/model.sdf\", name: \"item\", pose: {position: {x: 1.5, y: 0, z: 0.5}, orientation: {x: $OX, y: $OY, z: $OZ, w: $OW}}" > /dev/null
sleep 5

ros2 run ros_gz_bridge parameter_bridge --ros-args \
    -p config_file:="$PWD/sim/bridge.yaml" > /tmp/bridge_dump.log 2>&1 &
BR=$!
sleep 5

python3 scripts/dump_camera.py --out "$OUT" --frames 3

kill "$BR" 2>/dev/null || true
pkill -f parameter_bridge 2>/dev/null || true
pkill -f "ign gazebo" 2>/dev/null || true
echo "frames in $OUT"
