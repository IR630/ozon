#!/usr/bin/env bash
# Provision Ubuntu 22.04 (WSL2 or bare metal) with the project environment:
# ROS 2 Humble + Gazebo Fortress (ros_gz) + Python deps + docker-ce.
# Run as root from the repo root:  bash scripts/provision_wsl.sh
# In WSL the repo is reachable at /mnt/d/vano/ozon (adjust to your mount).
set -euxo pipefail

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y curl gnupg lsb-release locales
locale-gen en_US.UTF-8

# ROS 2 Humble apt repo
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

# Gazebo (OSRF) apt repo — Fortress binaries
curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

apt-get update
apt-get install -y \
    ros-humble-ros-base \
    ros-humble-ros-gz \
    ros-humble-cv-bridge \
    ignition-fortress \
    python3-colcon-common-extensions \
    python3-pip

pip3 install --no-cache-dir numpy scipy trimesh pytest ruff opencv-python-headless

# docker-ce: build/run the compose environment inside WSL (no Docker Desktop)
install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu jammy stable" \
    > /etc/apt/sources.list.d/docker.list
apt-get update
apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

# WSL: systemd so the docker service starts on boot
if grep -qi microsoft /proc/version && [ ! -f /etc/wsl.conf ]; then
    printf '[boot]\nsystemd=true\n' > /etc/wsl.conf
    echo "wsl.conf written: restart the distro (wsl --terminate <name>) to enable systemd"
fi

echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
echo "PROVISION DONE"
