# -*- coding: utf-8 -*-
"""Skeleton bring-up for the sorting cell (day 2, P1).

Starts the nodes that exist and run standalone after day 2:
  * ros_gz_bridge  — Gazebo camera topics -> ROS 2 (sim/bridge.yaml)
  * perception     — depth frames -> ItemMeasurement (src/perception_node.py)
  * classifier     — ItemMeasurement -> ItemClassification (src/classifier_node.py)

The camera->measurement->classification chain is live; controller_node
(/item/classification -> /pusher/cmd) lands on day 3 and completes the loop.

Prereqs (WSL / ROS 2 Humble): ros_gz_bridge installed, ros_msgs overlay built
and sourced. Run Gazebo separately (scripts spawn the world), then:
    ros2 launch launch/skeleton.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        parameters=[{"config_file": os.path.join(REPO_ROOT, "sim", "bridge.yaml")}],
        output="screen",
    )
    # src/ is not a ROS package: run the nodes as modules from the repo root so
    # `src.*` and the sourced `ros_msgs.msg` overlay both resolve on sys.path.
    perception = ExecuteProcess(
        cmd=["python3", "-m", "src.perception_node"],
        cwd=REPO_ROOT,
        output="screen",
    )
    classifier = ExecuteProcess(
        cmd=["python3", "-m", "src.classifier_node"],
        cwd=REPO_ROOT,
        output="screen",
    )
    return LaunchDescription([bridge, perception, classifier])
