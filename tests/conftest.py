import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

# ISOLATE THE TESTS' DDS GRAPH FROM ANY RUNNING SIMULATION.
#
# The node tests spin real rclpy nodes on real topics — /conveyor/cmd_vel,
# /pusher_c/cmd, /item/classification — and so does a live cell. With no
# ROS_DOMAIN_ID both land on domain 0, so a probe subscribed in a test receives the
# SIMULATOR's commands: the belt ramp reads 0.875 because that is where the running
# belt happened to be, and a "the pusher never fired" assertion sees the live cell's
# parking zeros. Running the suite beside a census turned 30 passed into 15 failed,
# and every failure looked like flakiness in our code.
#
# Set before rclpy is imported anywhere — rclpy reads it at init and never re-reads.
# `setdefault` so CI or an operator can still pin a domain deliberately.
os.environ.setdefault("ROS_DOMAIN_ID", "42")
