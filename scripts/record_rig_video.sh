#!/usr/bin/env bash
# Video of the THREE-HEAD rig: the scene with every camera VISIBLE, plus what each
# head sees, in one frame. Wraps run_stream.sh exactly like record_stream_video.sh
# does, with three differences that are the whole point of this script:
#
#   1. the world is cell_diverter_3cam.sdf, not the single-camera default;
#   2. BRIDGE_CONFIG is bridge_3cam.yaml — WITHOUT the sim/ prefix, because
#      launch/skeleton.launch.py joins the prefix itself and dump_item_frame.sh
#      does not. Getting this wrong sends the run to a one-camera world in
#      silence, and "the side heads see nothing" becomes the conclusion;
#   3. camera_side_props is spawned alongside the gantry: the camera models in
#      the world carry a <sensor> and no <visual>, so without the props the rig
#      is invisible in its own footage.
#
#   bash scripts/record_rig_video.sh <out.mp4> [slug:zone:gap_m ...]
#
# The props and the spectator are spawned into the RUNNING world, never committed
# into it, so no measurement run ever sees them (same contract as the gantry).
# The mp4 is NOT committed (GIT.md).
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

OUT=${1:?usage: record_rig_video.sh <out.mp4> [slug:zone:gap_m ...]}
shift
mkdir -p "$(dirname "$OUT")"

export WORLD=${WORLD:-sim/worlds/cell_diverter_3cam.sdf}
export BRIDGE_CONFIG=${BRIDGE_CONFIG:-bridge_3cam.yaml}
echo "=== rig video: WORLD=$WORLD BRIDGE_CONFIG=$BRIDGE_CONFIG ==="

bash scripts/run_stream.sh "$@" > runs/record_rig_run.log 2>&1 &
RUN=$!

# wait out run_stream's initial pkill of stale servers, then for the world
sleep 4
for _ in $(seq 1 40); do
    ign topic -l 2>/dev/null | grep -q "^/world/cell" && break
    sleep 0.5
done
sleep 1
for MODEL in spectator camera_gantry camera_side_props; do
    ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
        --reptype ignition.msgs.Boolean --timeout 5000 \
        --req "sdf_filename: \"$PWD/sim/models/$MODEL/model.sdf\", name: \"$MODEL\"" \
        > /dev/null
done
ros2 run ros_gz_bridge parameter_bridge \
    "/spectator/image@sensor_msgs/msg/Image[ignition.msgs.Image" \
    > runs/spectator_rig_bridge.log 2>&1 &
BRIDGE=$!
python3 scripts/save_rig_video.py --out "$OUT" --poster "${OUT%.mp4}_poster.png" &
SAVER=$!

# A FAIL episode is a NORMAL outcome; the exit code mirrors the verdict, so `wait`
# must not abort under errexit before cleanup (same reasoning as the other wrappers).
VERDICT=0
wait $RUN || VERDICT=$?
kill -INT $SAVER 2>/dev/null || true
sleep 2
kill $BRIDGE $SAVER 2>/dev/null || true
grep " -> " runs/record_rig_run.log || true
ls -lh "$OUT"
exit $VERDICT
