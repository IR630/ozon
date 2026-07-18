#!/usr/bin/env bash
# Video of a MULTI-ITEM stream episode (week 5, P1 hero): the money shot for the
# presentation — several items sorted back-to-back on the final diverter, backing
# the sustained-throughput claim visually. Wraps run_stream.sh the same way
# record_skeleton_video.sh wraps run_skeleton.sh: spawns the spectator camera
# (hero pose in sim/models/spectator/), bridges its frames to ROS, writes mp4 +
# poster via scripts/save_video.py. Playback is in sim time (save_video.py).
#
#   bash scripts/record_stream_video.sh <out.mp4> [slug:zone:gap_m ...]
#   # no specs -> run_stream.sh's default 3-item stream on cell_diverter.sdf
#
# The mp4 is NOT committed (GIT.md). errexit first + workspace guard before the
# source: same contract as run_skeleton.sh (see its header for the full story).
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

OUT=${1:?usage: record_stream_video.sh <out.mp4> [slug:zone:gap_m ...]}
shift
mkdir -p "$(dirname "$OUT")"

bash scripts/run_stream.sh "$@" > /tmp/record_stream_run.log 2>&1 &
RUN=$!

# wait out run_stream's initial pkill of stale servers, then for the world
sleep 4
for _ in $(seq 1 40); do
    ign topic -l 2>/dev/null | grep -q "^/world/cell" && break
    sleep 0.5
done
sleep 1
ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$PWD/sim/models/spectator/model.sdf\", name: \"spectator\"" > /dev/null
# visual-only prop: make the production camera visible in footage (see model.sdf)
ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$PWD/sim/models/camera_gantry/model.sdf\", name: \"camera_gantry\"" > /dev/null
ros2 run ros_gz_bridge parameter_bridge \
    "/spectator/image@sensor_msgs/msg/Image[ignition.msgs.Image" > /tmp/spectator_stream_bridge.log 2>&1 &
BRIDGE=$!
python3 scripts/save_video.py --topic /spectator/image --out "$OUT" \
    --poster "${OUT%.mp4}_poster.png" &
SAVER=$!

# A FAIL episode is a NORMAL outcome; the exit code mirrors the verdict, so `wait`
# must not abort under errexit before cleanup (same reasoning as the skeleton wrapper).
VERDICT=0
wait $RUN || VERDICT=$?
kill -INT $SAVER 2>/dev/null || true
sleep 2
kill $BRIDGE $SAVER 2>/dev/null || true
grep " -> " /tmp/record_stream_run.log || true
ls -lh "$OUT"
exit $VERDICT
