# Dev/runtime environment: ROS 2 Humble + Gazebo Fortress (docs/decisions.md).
# ros-humble-ros-gz in Humble targets Fortress — the LTS pair we standardize on.
# The tag is pinned by digest (Stage 25): "humble-desktop" is a moving target,
# and the organizers' server must build the image we validated, not today's.
FROM osrf/ros:humble-desktop@sha256:3d87cf339919a85cff7743ec9ba5e7ec81ccc26c9f722f1c7a6af5008dfdc128

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz \
    ros-humble-cv-bridge \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
COPY docker/pip-constraints.txt /tmp/pip-constraints.txt
# The constraints file pins every dependency to the validated versions
# (Stage 25). It also keeps numpy at the apt-built 1.x: ROS ships apt-built
# scipy/cv_bridge compiled against numpy 1.x; an unpinned OpenCV wheel would
# otherwise drag in numpy 2 and break those extensions (binary ABI).
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt -c /tmp/pip-constraints.txt

# repo is mounted at /ws (see docker-compose.yml)
WORKDIR /ws
ENV IGN_GAZEBO_RESOURCE_PATH=/ws/sim/models:/ws/sim/worlds
