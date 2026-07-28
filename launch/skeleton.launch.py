# -*- coding: utf-8 -*-
"""Bring up the complete single- or multi-item sorting pipeline.

The full v0.1 loop on one command:
  * ros_gz_bridge  — Gazebo <-> ROS 2 (camera, /clock, actuators; sim/bridge.yaml)
  * perception     — depth frames -> per-item measurements with stable IDs
  * classifier     — per-ID aggregation -> ItemClassification
  * controller     — per-ID schedules, belt, pusher/diverter commands and watchdogs

The actuator topics are the mechanism seam: cell.sdf interprets them as pusher
velocities, while the final cell_diverter.sdf interprets them as blade angles.
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
        # BRIDGE_CONFIG lets the three-head world bridge its extra depth streams
        # without the shipped single-camera census bridging topics it does not have.
        # PRODUCTION IS THE THREE-HEAD RIG (28.07, owner's call). The default moved
        # from bridge.yaml so the shipped entry points bridge the rig they run.
        # A one-head world with this config simply never publishes the side topics
        # and perception degrades to the top view bit-exactly (src/multiview.py),
        # so the fallback is `BRIDGE_CONFIG=bridge.yaml` — one variable, no code.
        parameters=[{"config_file": os.path.join(
                         REPO_ROOT, "sim",
                         os.environ.get("BRIDGE_CONFIG", "bridge_3cam.yaml")),
                     "use_sim_time": True}],
        output="screen",
    )
    # src/ is not a ROS package: run the nodes as modules from the repo root so
    # `src.*` and the sourced `ros_msgs.msg` overlay both resolve on sys.path.
    def node_cmd(mod):
        cmd = ["python3", "-m", f"src.{mod}"] + SIM_TIME
        # mechanism seam: the diverter world needs the blade held engaged and
        # fired with a sweep lead (run_skeleton.sh exports HOLD_S/FIRE_LEAD_S),
        # the pusher default needs nothing
        if mod == "controller_node":
            for env, param in (("HOLD_S", "hold_s"), ("FIRE_LEAD_S", "fire_lead_s"),
                               ("ENGAGE_CMD", "engage_cmd"), ("RETRACT_CMD", "retract_cmd"),
                               ("ESTOP_HOLD_MECHANISM", "estop_hold_mechanism")):
                if os.environ.get(env):
                    cmd += ["-p", f"{param}:={os.environ[env]}"]
        return cmd

    nodes = [
        ExecuteProcess(cmd=node_cmd(mod), cwd=REPO_ROOT, output="screen")
        for mod in ("perception_node", "classifier_node", "controller_node")
    ]
    return LaunchDescription([bridge, *nodes])
