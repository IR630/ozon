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

# The gate runs the SHIPPED rig, which since 28.07 is the three-head one.
export WORLD=sim/worlds/cell_diverter_3cam.sdf
export BRIDGE_CONFIG=bridge_3cam.yaml
export ITEM_MODEL_ROOT=sim/models/fixtures
export ROS_INSTALL_ROOT="$COLCON_ROOT/install"
# A three-head world boots ~40 s against run_skeleton.sh's one-head default wait of
# 30 s, and its cell cycle is ~59 s against ~27 s. Both budgets are raised here or
# the gate fails on the stopwatch rather than on the cell — the exact trap that made
# a healthy three-head census report 0/33 (docs/decisions.md 28.07).
export SOFT_START_TRIES=240
timeout 300 bash scripts/run_skeleton.sh box_300x200x200 B

export LOGDIR=/tmp/estop_stream_e2e
timeout 180 bash scripts/smoke_estop_stream.sh
