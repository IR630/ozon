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

export WORLD=sim/worlds/cell_diverter.sdf
export ITEM_MODEL_ROOT=sim/models/fixtures
export ROS_INSTALL_ROOT="$COLCON_ROOT/install"
timeout 120 bash scripts/run_skeleton.sh box_300x200x200 B
