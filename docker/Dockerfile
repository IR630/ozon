# Dev/runtime environment: ROS 2 Humble + Gazebo Fortress (docs/decisions.md).
# ros-humble-ros-gz in Humble targets Fortress — the LTS pair we standardize on.
FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz \
    ros-humble-cv-bridge \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
# numpy<2: ROS ships apt-built scipy/cv_bridge compiled against numpy 1.x;
# opencv wheels otherwise drag in numpy 2 and break them (binary ABI)
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt "numpy<2" opencv-python-headless

# repo is mounted at /ws (see docker-compose.yml)
WORKDIR /ws
ENV IGN_GAZEBO_RESOURCE_PATH=/ws/sim/models:/ws/sim/worlds
