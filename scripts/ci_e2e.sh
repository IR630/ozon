#!/usr/bin/env bash
# CI-only full contour smoke inside docker/Dockerfile's ROS + Gazebo image.
set -e
cd "$(dirname "$0")/.."

source /opt/ros/humble/setup.bash
COLCON_ROOT=/tmp/ozon_colcon
colcon --log-base "$COLCON_ROOT/log" build --base-paths ros_msgs \
    --build-base "$COLCON_ROOT/build" --install-base "$COLCON_ROOT/install" \
    --packages-select ros_msgs --event-handlers console_direct+
source "$COLCON_ROOT/install/setup.bash"

# The gate runs the SHIPPED rig, which since 30.07 is the TWO-head one.
export WORLD=sim/worlds/cell_diverter_2cam.sdf
export BRIDGE_CONFIG=bridge_2cam.yaml
export ITEM_MODEL_ROOT=sim/models/fixtures
export ROS_INSTALL_ROOT="$COLCON_ROOT/install"
# A multi-head world boots slower than run_skeleton.sh's one-head default wait of
# 30 s (measured: ~29 s for two heads, ~40 s for three). The budget is raised here
# or the gate fails on the stopwatch rather than on the cell — the exact trap that
# made a healthy three-head census report 0/33 (docs/decisions.md 28.07). Left at
# the three-head figure on purpose: it is a ceiling, and a two-head rig that boots
# faster loses nothing by being allowed more time.
export SOFT_START_TRIES=240
timeout 300 bash scripts/run_skeleton.sh box_300x200x200 B

export LOGDIR=/tmp/estop_stream_e2e
# 300, not 180. The e2e job went red on a loaded runner (02.08, run 30730053772,
# exit 124) with the SAME code that is green on the commits either side of it, and
# the log shows why the red is not a defect: the smoke reached its last meaningful
# line — `blade C after recovery gate: 0.000 rad`, i.e. the E-stop recovery gate
# had already passed — and only then hit the cap. That is the stopwatch failing,
# not the contour, and this project has twice drawn a false conclusion from an
# rc=124 before. The skeleton step above already carries 300 for the same reason
# (a multi-head world boots slower than the default wait); the E-stop stream runs
# the same world plus a stream and had less budget than the simpler step.
timeout 480 bash scripts/smoke_estop_stream.sh
