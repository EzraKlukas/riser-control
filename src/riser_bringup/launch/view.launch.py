"""Move the three angular sliders; lateral slide states follow kinematically."""

from pathlib import Path
import xml.etree.ElementTree as E
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share = Path(get_package_share_directory("riser_bringup"))
    description = xacro.process_file(
        str(share / "urdf/riser.urdf.xacro"), mappings={"backend": "none"}
    ).toxml()
    # GUI needs only the public coordinates, while robot_state_publisher gets the
    # full physical tree. Convert internal joints to fixed in the GUI copy.
    gui = E.fromstring(description)
    for j in gui.findall("joint"):
        if j.get("name") not in ["backboard_pitch", "foot_plate_pitch", "ml_angle"]:
            j.set("type", "fixed")
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": description}],
                remappings=[("joint_states", "/riser/physical_joint_states")],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[{"robot_description": E.tostring(gui, encoding="unicode")}],
            ),
            Node(package="riser_bringup", executable="physical_states.py"),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(share / "config/riser.rviz")],
            ),
        ]
    )
