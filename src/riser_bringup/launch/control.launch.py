"""RISER mock, Gazebo and G5 hardware control entry points."""

import os
from copy import deepcopy
from tempfile import NamedTemporaryFile

import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def setup(context):
    backend = LaunchConfiguration("backend").perform(context)
    selection = LaunchConfiguration("enabled_axes").perform(context)
    calibrate = LaunchConfiguration("calibrate").perform(context) == "true"
    if backend == "ethercat":
        launch = (
            Path(get_package_share_directory("riser_motion"))
            / "launch/bottom_motor.launch.py"
        )
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch)),
                launch_arguments={
                    "drive_backend": "real",
                    "enabled_axes": selection,
                    "controllers_file": LaunchConfiguration("g5_controllers_file"),
                    **({"load_motion_inactive": "true"} if calibrate else {}),
                }.items(),
            ),
        ]
    mode = LaunchConfiguration("command_mode").perform(context)
    human = LaunchConfiguration("human").perform(context) == "true"
    if human and backend != "gazebo":
        raise ValueError("The simulated participant requires backend:=gazebo")
    motion = backend == "gazebo" and (calibrate or LaunchConfiguration("motion").perform(context) == "true")
    if calibrate and backend == "mock":
        raise ValueError("Calibration requires Gazebo or EtherCAT travel switches")
    if mode.startswith("actuator_") and backend != "gazebo":
        raise ValueError("Actuator command modes currently require the Gazebo backend")
    share = Path(get_package_share_directory("riser_bringup"))
    world = str(share / "worlds/riser_world.sdf")
    plot = LaunchConfiguration("plot").perform(context) == "true"
    wrench_input = LaunchConfiguration("wrench_input").perform(context) == "true"
    sim = backend == "gazebo"
    motor_axes = ("top", "torso", "hip", "bottom")
    enabled = [axis.strip() for axis in selection.split(",")] if sim and selection else motor_axes
    if len(set(enabled)) != len(enabled) or not set(enabled) <= set(motor_axes):
        raise ValueError("Select one to four unique enabled_axes from top, torso, hip, bottom")
    masked = sim and len(enabled) < len(motor_axes)
    available = {f"{axis}_motor" for axis in enabled}
    if "top" in enabled:
        available.add("backboard_pitch")
    if "bottom" in enabled:
        available.add("foot_plate_pitch")
    if {"torso", "hip"} <= set(enabled):
        available.add("ml_angle")
    chosen = f"riser_{mode.removeprefix('angular_')}_controller"
    spawn_chosen = True
    controllers = LaunchConfiguration("controllers_file").perform(context)
    cleanup = []
    inactive = []
    if motion or masked:
        config = yaml.safe_load(Path(controllers).read_text())
    if motion:
        motion_config = yaml.safe_load(
            Path(LaunchConfiguration("g5_controllers_file").perform(context)).read_text()
        )
        manager = config["controller_manager"]["ros__parameters"]
        manager["calibration_controller"] = {"type": "riser_calibration/CalibrationController"}
        config["calibration_controller"] = {
            "ros__parameters": {"speed_m_s": .045, "max_velocity_m_s": .05, "max_travel_m": .70}
        }
        manager["g5_plot"] = deepcopy(manager["joint_state_broadcaster"])
        config["g5_plot"] = {
            "ros__parameters": {"update_rate": 25, "use_urdf_to_filter": False, "use_local_topics": True}
        }
        if human:
            participant = Path(get_package_share_directory("riser_human_balance"))
            pendulum = yaml.safe_load((participant / "config/pendulum.yaml").read_text())
            config.setdefault("inverted_pendulum", {}).setdefault("ros__parameters", {}).update(
                pendulum["inverted_pendulum"]["ros__parameters"]
            )
        balance = config.setdefault("inverted_pendulum", {}).setdefault("ros__parameters", {})
        balance["read_time_interface"] = "RiserSystem/read_time"
        balance_axis = {"footplate": "foot_plate_pitch", "backboard": "backboard_pitch"}.get(
            balance.get("mode", "footplate")
        )
        inactive = ["calibration_controller"]
        if balance_axis is None or balance_axis in available:
            inactive.append("inverted_pendulum")
        types = motion_config["controller_manager"]["ros__parameters"]
        for joint, profile in [
            ("bottom_motor", "motor_profile"), ("top_motor", "top_profile"),
            ("torso_motor", "torso_profile"), ("hip_motor", "hip_profile"),
            ("foot_plate_pitch", "angle_profile"), ("backboard_pitch", "backboard_profile"),
            ("ml_angle", "ml_profile"),
        ]:
            if joint not in available:
                continue
            for mode_name in ("velocity", "position"):
                name = profile if mode_name == "velocity" else profile.replace("_profile", "_position_profile")
                manager[name] = deepcopy(types["motor_profile"])
                config[name] = deepcopy(motion_config["motor_profile"])
                params = config[name]["ros__parameters"]
                params.update(
                    joint=joint, command_mode=mode_name,
                    read_time_interface="RiserSystem/read_time",
                )
                if joint != "foot_plate_pitch":
                    params.update(max_velocity_m_s=.03, max_travel_m=.05, max_acceleration_m_s2=.12)
                inactive.append(name)
    if masked:
        for name in ("riser_position_controller", "riser_velocity_controller",
                     "riser_actuator_position_controller", "riser_actuator_velocity_controller"):
            params = config.get(name, {}).get("ros__parameters", {})
            if "joints" in params:
                params["joints"] = [joint for joint in params["joints"] if joint in available]
                if name == chosen:
                    spawn_chosen = bool(params["joints"])
    if motion or masked:
        with NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as stream:
            yaml.safe_dump(config, stream)
            controllers = stream.name
        cleanup.append(RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda context: Path(controllers).unlink(missing_ok=True))
        ])))
    plot_telemetry = LaunchConfiguration("plot_telemetry").perform(context) == "true"
    # having parsed command line inputs, we parse urdf, first conforming the xml to the inputs.
    description = xacro.process_file(
        str(share / "urdf/riser.urdf.xacro"),
        mappings={
            "backend": backend,
            "controllers_file": controllers,
            "command_mode": mode,
            "enabled_axes": ",".join(enabled),
            "control_coordinates_file": LaunchConfiguration("coordinates_file").perform(
                context
            ),
            "force_plate_rate": LaunchConfiguration("force_plate_rate").perform(
                context
            ),
        },
    ).toxml()
    actions = cleanup + [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description, "use_sim_time": sim}],
            remappings=[("joint_states", "/riser/physical_joint_states")],
        )
    ]
    if sim:
        gz = Path(get_package_share_directory("ros_gz_sim"))
        plugin_packages = ["riser_gazebo", "riser_subject_simulation"]
        if human:
            plugin_packages.append("riser_human_balance")
        plugin_dirs = [
            str(Path(get_package_share_directory(package)).parents[1] / "lib")
            for package in plugin_packages
        ]
        actions += [
            SetEnvironmentVariable(
                "GZ_SIM_SYSTEM_PLUGIN_PATH",
                ":".join(
                    plugin_dirs + [os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")]
                ),
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
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
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
        params = {}
        actions.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[params, controllers],
                remappings=[("robot_description", "/robot_description")],
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
    # Load the ordinary broadcaster before controller-exported states exist,
    # then activate the wrench provider before its dedicated consumer.
    names = ["joint_state_broadcaster"]
    if spawn_chosen and (not sim or (not motion and mode.startswith("actuator_"))):
        names.append(chosen)
    if motion:
        names.append("g5_plot")
    elif sim and spawn_chosen and not mode.startswith("actuator_"):
        inactive.append(chosen)
    if sim:
        names += ["ankle_wrench", "ankle_wrench_state_broadcaster"]
        if plot or plot_telemetry:
            names.append("ankle_wrench_plot_broadcaster")
            actions.append(
                Node(
                    package="riser_bringup",
                    executable="ankle_plot_wrenches.py",
                )
            )
    args = names + [
        "--controller-manager",
        "/controller_manager",
        "--controller-manager-timeout",
        "120",
        "--param-file",
        controllers,
    ]
    actions.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=args,
            output="screen",
        )
    )

    if inactive:
        actions.append(Node(
            package="controller_manager", executable="spawner",
            arguments=[*inactive, "--inactive", "--controller-manager-timeout", "120",
                       "--param-file", controllers], output="screen",
        ))

    if plot:
        actions.append(Node(package="plotjuggler", executable="plotjuggler"))
    if wrench_input:
        actions.append(
            Node(
                package="riser_subject_simulation",
                executable="simulation_wrench_input",
                arguments=["--gui"],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "g5_controllers_file",
                default_value=str(
                    Path(get_package_share_directory("riser_motion"))
                    / "config/controllers.yaml"
                ),
                description="Motion profile configuration for Gazebo and EtherCAT.",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=str(
                    Path(get_package_share_directory("riser_bringup"))
                    / "config/controllers.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "plot_telemetry",
                default_value="false",
                choices=["true", "false"],
                description="Enable 25 Hz ankle-wrench plotting topics in Gazebo with plot:=false.",
            ),
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
            DeclareLaunchArgument("force_plate_rate", default_value="1000"),
            DeclareLaunchArgument(
                "enabled_axes", default_value="",
                description="Comma-separated motor names. Gazebo defaults to all four.",
            ),
            DeclareLaunchArgument("motion", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument(
                "human", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "calibrate", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "backend", default_value="mock", choices=["mock", "gazebo", "ethercat"]
            ),
            DeclareLaunchArgument(
                "command_mode",
                default_value="position",
                choices=[
                    "position",
                    "velocity",
                    "angular_position",
                    "angular_velocity",
                    "actuator_position",
                    "actuator_velocity",
                ],
            ),
            DeclareLaunchArgument(
                "plot", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "wrench_input", default_value="true", choices=["true", "false"]
            ),
            OpaqueFunction(function=setup),
        ]
    )
