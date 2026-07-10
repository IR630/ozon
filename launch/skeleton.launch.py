# -*- coding: utf-8 -*-
"""Skeleton bring-up for the sorting cell (day 2, P1).

Starts the nodes that exist and run standalone after day 2:
  * ros_gz_bridge  — Gazebo camera topics -> ROS 2 (sim/bridge.yaml)
  * classifier     — ItemMeasurement -> ItemClassification (src/classifier_node.py)

This is the seed of the day-3 end-to-end skeleton, not the whole loop:
perception_node (camera -> /item/measurement) and controller_node
(/item/classification -> /pusher/cmd) are added on day 3 when their node
wrappers are built for integration. Today the launch proves the two real
nodes come up together on one command.

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
    # src/ is not a ROS package: run the node as a module from the repo root so
    # `src.*` and the sourced `ros_msgs.msg` overlay both resolve on sys.path.
    classifier = ExecuteProcess(
        cmd=["python3", "-m", "src.classifier_node"],
        cwd=REPO_ROOT,
        output="screen",
    )
    return LaunchDescription([bridge, classifier])
