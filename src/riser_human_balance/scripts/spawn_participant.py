#!/usr/bin/env python3
"""Spawn the optional participant with its rigid feet parallel to the plate."""

import argparse
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pulse-nm", type=float, default=0.0,
                        help="Brief ankle torque perturbation at release (N m)")
    args = parser.parse_args()
    if not -50.0 <= args.pulse_nm <= 50.0:
        parser.error("--pulse-nm must be between -50 and 50")

    rclpy.init()
    node = rclpy.create_node("spawn_participant")
    try:
        ready, feedback = wait_for_message(
            JointState, node, "/riser/physical_joint_states",
            qos_profile=qos_profile_sensor_data, time_to_wait=5.0,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not ready or "foot_plate_pitch" not in feedback.name:
        parser.error("No footplate feedback. Start and home the Gazebo robot first.")
    axis = feedback.name.index("foot_plate_pitch")
    pitch = feedback.position[axis]
    if not (abs(pitch) <= 0.01 and abs(feedback.velocity[axis]) <= 0.005):
        parser.error("Wait for the footplate to settle within 0.01 rad of flat.")

    share = Path(get_package_share_directory("riser_human_balance"))
    sdf = ET.parse(share / "models/participant.sdf").getroot()
    sdf.find("model/plugin/pulse_nm").text = str(args.pulse_nm)
    # The common ankle pivot is 0.6477 m above the Gazebo floor
    # (riser_bringup/urdf/model/kinematics.yaml).
    return subprocess.run(
        ["ros2", "run", "ros_gz_sim", "create", "-world", "riser_world",
         "-name", "riser_participant", "-z", "0.6477", "-P", str(pitch),
         "-string", ET.tostring(sdf, encoding="unicode")],
        check=False,
    ).returncode


if __name__ == "__main__":
    raise SystemExit(main())
