#!/usr/bin/env bash
# Fast framing preview for the presentation hero shot (P1, week 5): spawn the
# world + spectator camera and grab ONE frame as PNG — WITHOUT running a routing
# episode. Iterating on the spectator pose costs seconds instead of a full render.
#
#   SPECTATOR_POSE="x y z r p yaw" bash scripts/frame_spectator.sh runs/frames/a.png
#
# Reuses the proven spawn/bridge/poster sequence from record_skeleton_video.sh.
# For the final hero footage (item in motion) use record_skeleton_video.sh; this
# is only for composition. Output goes under runs/ (gitignored).
set -e
cd "$(dirname "$0")/.."
export LIBGL_ALWAYS_SOFTWARE=1
ROS_INSTALL_ROOT=${ROS_INSTALL_ROOT:-install}
if [ ! -f "$ROS_INSTALL_ROOT/setup.bash" ]; then
    echo "ABORT: ROS workspace is not built ($ROS_INSTALL_ROOT/setup.bash is missing) — run:" >&2
    echo "    colcon build --packages-select ros_msgs" >&2
    exit 1
fi
source /opt/ros/humble/setup.bash
source "$ROS_INSTALL_ROOT/setup.bash"

OUT=${1:?usage: SPECTATOR_POSE="x y z r p yaw" frame_spectator.sh out.png}
POSE=${SPECTATOR_POSE:?set SPECTATOR_POSE="x y z r p yaw"}
mkdir -p "$(dirname "$OUT")"

pkill -f 'ign gazebo' 2>/dev/null || true
sleep 1
ign gazebo -s -r -v 0 sim/worlds/cell.sdf > /tmp/frame_gz.log 2>&1 &
GZ=$!
for _ in $(seq 1 40); do
    ign topic -l 2>/dev/null | grep -q "^/world/cell" && break
    sleep 0.5
done
sleep 1

# inject the requested pose into a temp copy of the spectator model. The
# spectator model has exactly one <pose> (the model pose; the sensor has none),
# so replace whatever pose is committed — robust to the SDF being re-aimed.
sed -E "s|<pose>[^<]*</pose>|<pose>$POSE</pose>|" \
    sim/models/spectator/model.sdf > /tmp/spectator_frame.sdf
ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"/tmp/spectator_frame.sdf\", name: \"spectator\"" > /dev/null

ros2 run ros_gz_bridge parameter_bridge \
    "/spectator/image@sensor_msgs/msg/Image[ignition.msgs.Image" > /tmp/frame_bridge.log 2>&1 &
BRIDGE=$!
python3 scripts/save_video.py --topic /spectator/image --out /tmp/frame.mp4 \
    --poster "$OUT" &
SAVER=$!

sleep 5
kill -INT $SAVER 2>/dev/null || true
sleep 1
kill $BRIDGE $SAVER $GZ 2>/dev/null || true
pkill -f 'ign gazebo' 2>/dev/null || true
ls -lh "$OUT"
