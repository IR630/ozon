#!/usr/bin/env bash
# Video of the end-to-end skeleton run (day 3, P5): wraps run_skeleton.sh,
# spawns the spectator camera (sim/models/spectator/) into the freshly
# started world, bridges its frames to ROS and writes an mp4 + poster PNG
# via scripts/save_video.py. Playback is in sim time (see save_video.py).
#
#   bash scripts/record_skeleton_video.sh <slug> <B|C|D> [out.mp4]
#
# The mp4 is NOT committed (GIT.md: no large binaries in the repo) — upload
# to the organizers' cloud, link in README. The poster PNG may go to the
# report. Exit code mirrors the run verdict, so a FAIL run fails the script.
# errexit first, and a loud check before sourcing install/setup.bash — same
# defect/fix as run_skeleton.sh (see that file's header for the full story).
# This script had no errexit at all before this fix.
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

SLUG=${1:?usage: record_skeleton_video.sh <slug> <B|C|D> [out.mp4]}
EXPECT=${2:?expected zone B|C|D}
OUT=${3:-docs/report/video/skeleton_${SLUG}_${EXPECT}.mp4}
mkdir -p "$(dirname "$OUT")"

# the spectator render drops RTF ~10x under llvmpipe — widen the wall-clock
# verdict window accordingly (the contour itself runs on sim time, unaffected).
# Overridable: helmet→B creeps to x=4.2 and needs a wider window than 300
# under recording (18.07: FAIL at 300 with body x=3.954 — budget, not physics).
export RUN_SKELETON_POLL_ITERS=${RUN_SKELETON_POLL_ITERS:-300}
bash scripts/run_skeleton.sh "$SLUG" "$EXPECT" > /tmp/record_run.log 2>&1 &
RUN=$!

# wait out run_skeleton's initial pkill of stale servers, then for the world
sleep 4
for _ in $(seq 1 40); do
    ign topic -l 2>/dev/null | grep -q "^/world/cell" && break
    sleep 0.5
done
sleep 1
ign service -s /world/cell/create --reqtype ignition.msgs.EntityFactory \
    --reptype ignition.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: \"$PWD/sim/models/spectator/model.sdf\", name: \"spectator\"" > /dev/null
ros2 run ros_gz_bridge parameter_bridge \
    "/spectator/image@sensor_msgs/msg/Image[ignition.msgs.Image" > /tmp/spectator_bridge.log 2>&1 &
BRIDGE=$!
python3 scripts/save_video.py --topic /spectator/image --out "$OUT" \
    --poster "${OUT%.mp4}_poster.png" &
SAVER=$!

# A FAIL episode is a NORMAL outcome here — the exit code mirrors the verdict — so
# `wait` must not abort the script now that errexit is armed: that would skip the
# cleanup below and leave the mp4 unclosed and the bridge/saver orphaned.
VERDICT=0
wait $RUN || VERDICT=$?
# SIGINT (not SIGTERM straight away) so the writer releases and the mp4 closes.
# `|| true`: by this point the saver has usually already exited, and killing a dead
# PID returns non-zero — under errexit that would kill the script on the SUCCESS
# path, before it ever reports the verdict.
kill -INT $SAVER 2>/dev/null || true
sleep 2
kill $BRIDGE $SAVER 2>/dev/null || true
grep " -> " /tmp/record_run.log || true
ls -lh "$OUT"
exit $VERDICT
