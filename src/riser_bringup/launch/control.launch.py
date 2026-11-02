"""Humble/Fortress simulation and mock angular control interfaces."""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def setup(context):
    backend = LaunchConfiguration("backend").perform(context)
    mode = LaunchConfiguration("command_mode").perform(context)
    share = Path(get_package_share_directory("riser_bringup"))
    world = LaunchConfiguration("world").perform(context) or str(
        share / "worlds/riser_world.sdf"
    )
    plot = LaunchConfiguration("plot").perform(context) == "true";
    wrench_ui = LaunchConfiguration("wrench_ui").perform(context) == "true";
    sim = backend == "gazebo"
    controllers = str(share / "config/controllers.yaml")
    # having parsed command line inputs, we parse urdf, first conforming the xml to the inputs.
    description = xacro.process_file(
        str(share / "urdf/riser.urdf.xacro"),
        mappings={
            "backend": backend,
            "controllers_file": controllers,
            "command_mode": mode,
            "control_coordinates_file": LaunchConfiguration("coordinates_file").perform(
                context
            ),
            "force_plate_rate": LaunchConfiguration("force_plate_rate").perform(
                context
            ),
        },
    ).toxml()
    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description, "use_sim_time": sim}],
            remappings=[("joint_states", "/riser/physical_joint_states")],
        )
    ]
    if sim:
        gz = Path(get_package_share_directory("ros_gz_sim"))
        plugin_dir = str(
            Path(get_package_share_directory("riser_gazebo")).parents[1] / "lib"
        )
        actions += [
            SetEnvironmentVariable(
                "IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
                plugin_dir + ":" + os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", ""),
            )
        ]  # tells gazebo sim where to search for system plugin shared libraries at runtime.
        actions += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gz / "launch/gz_sim.launch.py")),
                launch_arguments={
                    "gz_args": "-r "
                    + (
                        "-s "
                        if LaunchConfiguration("headless").perform(context) == "true"
                        else ""
                    )
                    + "-v 2 "
                    + world
                }.items(),
            ),  # embeds a launch file from inside this launch file.
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-topic", "robot_description", "-name", "riser"],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
            ),
        ]
        actions += [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[
                    {"config_file": str(share / "config/force_plate_bridge.yaml")}
                ],
            ),
            Node(package="riser_bringup", executable="force_plate_frames.py"),
        ]
    else:
        params = {"robot_description": description}
        if backend == "ethercat":
            params["hardware_components_initial_state"] = {
                "unconfigured": ["RiserSystem"]
            }
        actions.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[params, controllers],
                output="screen",
            )
        )
        actions.append(
            Node(
                package="riser_bringup",
                executable="physical_states.py",
                parameters=[{"use_sim_time": False}],
            )
        )
    chosen = f"riser_{mode}_controller"
    for name in ["joint_state_broadcaster", chosen]:
        args = [
            name,
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ]
        # Hardware starts inactive until the boundary can be configured explicitly.
        if backend == "ethercat":
            args.append("--inactive")
        actions.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=args,
                output="screen",
            )
        )

    if plot:
        actions.append(
            Node(
                package='plotjuggler',
                executable='plotjuggler'
            )
        )
    if wrench_ui:
        actions.append(Node(package="riser_subject_simulation", executable="simulation_wrench_input",
            arguments=["--tui"], parameters=[{"use_sim_time": True}], output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "coordinates_file",
                default_value=str(
                    Path(get_package_share_directory("riser_bringup"))
                    / "config/control_coordinates.json"
                ),
            ),
            DeclareLaunchArgument(
                "headless", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument("world", default_value=""),
            DeclareLaunchArgument("force_plate_rate", default_value="500"),
            DeclareLaunchArgument(
                "backend", default_value="mock", choices=["mock", "gazebo", "ethercat"]
            ),
            DeclareLaunchArgument(
                "command_mode",
                default_value="position",
                choices=["position", "velocity"],
            ),
            DeclareLaunchArgument(
                "plot",
                default_value="true",
                choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "wrench_ui",
                default_value="true",
                choices=["true", "false"]
            ),
            OpaqueFunction(function=setup),
        ]
    )
