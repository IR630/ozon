# -*- coding: utf-8 -*-
"""Skeleton bring-up for the sorting cell (days 2-3).

The full v0.1 loop on one command:
  * ros_gz_bridge  — Gazebo <-> ROS 2 (camera, /clock, actuators; sim/bridge.yaml)
  * perception     — depth frames -> ItemMeasurement (src/perception_node.py)
  * classifier     — ItemMeasurement -> ItemClassification (src/classifier_node.py)
  * controller     — classification -> belt soft-start + pusher fire (src/controller_node.py)

All nodes run on use_sim_time: camera stamps are Gazebo sim time and the
controller's dead-reckoning must tick on the same clock (/clock bridge).

Prereqs (WSL / ROS 2 Humble): ros_gz_bridge installed, ros_msgs overlay built
and sourced. Run Gazebo separately (scripts spawn the world), then:
    ros2 launch launch/skeleton.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM_TIME = ["--ros-args", "-p", "use_sim_time:=true"]


def generate_launch_description():
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        parameters=[{"config_file": os.path.join(REPO_ROOT, "sim", "bridge.yaml"),
                     "use_sim_time": True}],
        output="screen",
    )
    # src/ is not a ROS package: run the nodes as modules from the repo root so
    # `src.*` and the sourced `ros_msgs.msg` overlay both resolve on sys.path.
    nodes = [
        ExecuteProcess(cmd=["python3", "-m", f"src.{mod}"] + SIM_TIME,
                       cwd=REPO_ROOT, output="screen")
        for mod in ("perception_node", "classifier_node", "controller_node")
    ]
    return LaunchDescription([bridge, *nodes])
