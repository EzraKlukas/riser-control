"""Named rotary G5 axes sharing one master and the commissioned CSV scaling."""

from copy import deepcopy
from tempfile import NamedTemporaryFile
import yaml
from math import tau
from pathlib import Path
from xml.sax.saxutils import escape
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    Shutdown,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# R88M-K40030L-BS2: 20-bit incremental encoder (Omron G5 lineup/specification).
# reports/G5_parameter_reference.html: Pn001=0, Pn748=1, Pn749=1.
# Motor feedback + 1:1 electronic gear => one command unit per encoder count.
MOTOR_ENCODER_COUNTS_PER_REV = 1 << 20
COMMAND_UNITS_PER_MOTOR_RAD = MOTOR_ENCODER_COUNTS_PER_REV / tau


STATES = "position velocity ready reverse_limit forward_limit digital_inputs feedback_valid statusword controlword operating_mode fault_code torque_raw torque_nm brake_release_request brake_output_feedback physical_brake discovered host_fault"


def setup(context):
    keys = [
        "master_index",
        "alias",
        "position",
        "product_code",
        "position_units_per_rad",
        "velocity_units_per_rad_s",
        "max_velocity_rad_s",
        "max_travel_rad",
        "sync0_shift_ns",
        "commissioning_disabled",
    ]
    values = {key: LaunchConfiguration(key).perform(context) for key in keys}
    profile = yaml.safe_load(
        Path(LaunchConfiguration("axes_file").perform(context)).read_text()
    )
    backend = LaunchConfiguration("drive_backend").perform(context)
    selection = LaunchConfiguration("enabled_axes").perform(context)
    enabled = selection.split(",") if selection else profile["enabled"]
    if (
        not enabled
        or len(enabled) != len(set(enabled))
        or not set(enabled) <= {"bottom", "top", "torso", "hip"}
    ):
        raise ValueError("Select one to four unique named axes")
    values["configured_axes"] = ",".join(profile["axes"])
    values["enabled_axes"] = ",".join(enabled)
    config = yaml.safe_load(
        Path(LaunchConfiguration("controllers_file").perform(context)).read_text()
    )
    manager_params = config["controller_manager"]["ros__parameters"]
    if backend == "real":
        for key, value in {
            "cpu_affinity": 3,
            "thread_priority": 80,
            "lock_memory": True,
        }.items():
            manager_params.setdefault(key, value)
    template = config.pop("motor_profile")
    controller_type = manager_params.pop("motor_profile")
    links, joints, resources, controllers = [], [], [], []
    states = STATES.split()
    for name in enabled:
        axis = dict(profile["axes"][name])
        for key in ["position", "alias"]:
            if name == "bottom" and values[key]:
                axis[key] = int(values[key], 0)
        joint = name + "_motor"
        for key, value in axis.items():
            if backend == "real" and value is None:
                raise ValueError(f"Missing commissioned {name} {key}")
            values[joint + "_" + key] = str(value)
        links.append(f'<link name="{name}_shaft"/>')
        joints.append(
            f'<joint name="{joint}" type="continuous"><parent link="base"/><child link="{name}_shaft"/><axis xyz="0 0 1"/></joint>'
        )
        initial = {
            "ready": 2,
            "feedback_valid": 1,
            "discovered": 1,
            "physical_brake": -1,
            "brake_output_feedback": -1,
        }
        interfaces = "".join(
            f'<state_interface name="{key}"><param name="initial_value">{initial.get(key, 0)}</param></state_interface>'
            if backend == "mock"
            else f'<state_interface name="{key}"/>'
            for key in states
        )
        resources.append(
            f'<joint name="{joint}"><command_interface name="velocity"/>{interfaces}</joint>'
        )
        controller = "motor_profile" if name == "bottom" else name + "_profile"
        controllers.append(controller)
        manager_params[controller] = deepcopy(controller_type)
        config[controller] = deepcopy(template)
        config[controller]["ros__parameters"].setdefault("jog_mode", True)
        config[controller]["ros__parameters"].update(
            joint=joint,
            max_velocity_rad_s=float(values["max_velocity_rad_s"]),
            max_travel_rad=float(values["max_travel_rad"]),
        )
    if not set(profile["axes"]) <= {"bottom", "top", "torso", "hip"}:
        raise ValueError("Unknown configured axis")
    for name, axis in profile["axes"].items():
        if name not in enabled:
            resources.append(
                f'<sensor name="{name}_excluded"><state_interface name="discovered"><param name="initial_value">0</param></state_interface></sensor>'
            )
        if name not in enabled and axis.get("serial"):
            for key in ("position", "serial", "alias"):
                values[name + "_motor_" + key] = str(axis[key])
    parameters = "".join(
        f'<param name="{key}">{escape(value)}</param>'
        for key, value in values.items()
        if value
    )
    driver = (
        "riser_hardware/BottomCsvSystem"
        if backend == "real"
        else "mock_components/GenericSystem"
    )
    description = f"""<robot name="g5_bench"><link name="base"/>{"".join(links + joints)}
      <ros2_control name="BottomMotor" type="system"><hardware><plugin>{driver}</plugin>{parameters}
      <param name="calculate_dynamics">true</param><param name="cycle_ns">1000000</param></hardware>
      {"".join(resources)}</ros2_control></robot>"""
    broadcaster = config["joint_state_broadcaster"]["ros__parameters"]
    broadcaster.pop("joints", None)
    broadcaster.pop("interfaces", None)
    manager_params["g5_plot"] = deepcopy(manager_params["joint_state_broadcaster"])
    config["g5_plot"] = {
        "ros__parameters": dict(
            deepcopy(broadcaster), update_rate=25, use_local_topics=True
        )
    }
    with NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as file:
        yaml.safe_dump(config, file)
        config = file.name
    publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(description, value_type=str)}],
        output="screen",
    )

    def manager_exit(event, context):
        if context.is_shutdown:
            return []
        if event.returncode:
            raise RuntimeError(
                f"G5 controller manager exited with status {event.returncode}"
            )
        return [Shutdown(reason="G5 controller manager exited")]

    manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[config],
        additional_env={"MALLOC_ARENA_MAX": "2"} if backend == "real" else {},
        remappings=[("robot_description", "/robot_description")],
        output="screen",
        on_exit=manager_exit,
    )
    nodes = [
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    OpaqueFunction(
                        function=lambda context: Path(config).unlink(missing_ok=True)
                    )
                ]
            )
        ),
        publisher,
        manager,
    ]
    if LaunchConfiguration("feedback").perform(context) == "true":
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "g5_plot",
                    "--param-file",
                    config,
                ],
                output="screen",
            )
        )
    if (
        values["commissioning_disabled"] == "0"
        or LaunchConfiguration("load_motion_inactive").perform(context) == "true"
    ):
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[*controllers, "--inactive", "--param-file", config],
                output="screen",
            )
        )
    return nodes


def generate_launch_description():
    args = [DeclareLaunchArgument(key, default_value="0") for key in ["master_index"]]
    args += [
        DeclareLaunchArgument(
            "load_motion_inactive", default_value="false", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "feedback", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "drive_backend", default_value="real", choices=["real", "mock"]
        ),
        DeclareLaunchArgument(
            "axes_file",
            default_value=str(
                Path(get_package_share_directory("riser_motion_bringup"))
                / "config/axes.yaml"
            ),
        ),
        DeclareLaunchArgument("enabled_axes", default_value=""),
        DeclareLaunchArgument("position", default_value=""),
        DeclareLaunchArgument("alias", default_value=""),
        DeclareLaunchArgument("sync0_shift_ns", default_value="650000"),
    ]
    args += [
        DeclareLaunchArgument(
            "commissioning_disabled", default_value="1", choices=["0", "1"]
        )
    ]
    args += [
        DeclareLaunchArgument(key, default_value=str(COMMAND_UNITS_PER_MOTOR_RAD))
        for key in ["position_units_per_rad", "velocity_units_per_rad_s"]
    ]
    args += [DeclareLaunchArgument("product_code", default_value="0x00000004")]
    args += [
        DeclareLaunchArgument("max_velocity_rad_s", default_value="0.1047197551"),
        DeclareLaunchArgument("max_travel_rad", default_value="0.05"),
    ]
    args += [
        DeclareLaunchArgument(
            "controllers_file",
            default_value=str(
                Path(get_package_share_directory("riser_motion_bringup"))
                / "config/controllers.yaml"
            ),
        )
    ]
    return LaunchDescription(args + [OpaqueFunction(function=setup)])
