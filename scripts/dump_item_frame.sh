#!/usr/bin/env bash
# Boot Gazebo + the ros_gz bridge, spawn ONE item at rest UNDER the camera
# (x=1.5, belt stationary — no controller), dump depth/RGB PNGs. For offline
# perception debugging (day 4: vertical-section K on real depth frames).
#
#   bash scripts/dump_item_frame.sh <slug> [outdir]
#   ORIENT_{X,Y,Z,W}=... bash scripts/dump_item_frame.sh <slug>   # seeded pose
#   SPAWN_X=2.3 FRAMES=1 bash scripts/dump_item_frame.sh box_300x200x200 # partial view
#   WORLD=sim/worlds/cell_diverter.sdf bash scripts/dump_item_frame.sh <slug>
# The WORLD seam mirrors run_skeleton.sh: day-5 check that the diverter world's
# parked blades stay out of the camera frame (they broke perception when in it).
cd "$(dirname "$0")/.."
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
source install/setup.bash
set -e

SLUG=${1:?usage: dump_item_frame.sh <slug> [outdir]}
OUT=${2:-/tmp/frames_$SLUG}
FRAMES=${FRAMES:-3}
OX=${ORIENT_X:-0}; OY=${ORIENT_Y:-0}; OZ=${ORIENT_Z:-0}; OW=${ORIENT_W:-1}
# Spawn height/offset seam, mirroring run_skeleton.sh: a seeded pose must rest at
# its pose-dependent height (spawn_orientations.spawn_pose_for_mesh_m), else a
# fixed z=0.5 buries turned/tall items (helmet 0.66, plate 0.60) inside the belt
# and the solver ejects them — the dumped frame would then not be the census pose.
SPAWN_X=${SPAWN_X:-1.5}; SPAWN_Z=${SPAWN_Z:-0.5}; SPAWN_Y=${SPAWN_Y:-0}

pkill -f "ign gazebo" 2>/dev/null || true
pkill -f parameter_bridge 2>/dev/null || true
sleep 2

WORLD=${WORLD:-sim/worlds/cell.sdf}
ign gazebo -s -r -v 0 "$WORLD" > /tmp/gz_dump.log 2>&1 &
sleep 10

# Spawn at the requested X and let the item settle; the default is directly
# under the camera. An explicit near-border X captures the partial-visibility
# slice. The belt is never commanded, so the item stays in the camera window.
ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$PWD/sim/models/items/$SLUG/model.sdf\", name: \"item\", pose: {position: {x: $SPAWN_X, y: $SPAWN_Y, z: $SPAWN_Z}, orientation: {x: $OX, y: $OY, z: $OZ, w: $OW}}" > /dev/null
sleep 5

ros2 run ros_gz_bridge parameter_bridge --ros-args \
    -p config_file:="$PWD/sim/bridge.yaml" > /tmp/bridge_dump.log 2>&1 &
BR=$!
sleep 5

python3 scripts/dump_camera.py --out "$OUT" --frames "$FRAMES"

kill "$BR" 2>/dev/null || true
pkill -f parameter_bridge 2>/dev/null || true
pkill -f "ign gazebo" 2>/dev/null || true
echo "frames in $OUT"
