"""Single rotary G5 motor with snapshot-derived scaling and required motion bounds."""
from math import tau
from pathlib import Path
from xml.sax.saxutils import escape
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# R88M-K40030L-BS2: 20-bit incremental encoder (Omron G5 lineup/specification).
# reports/G5_parameter_reference.html: Pn001=0, Pn748=1, Pn749=1.
# Motor feedback + 1:1 electronic gear => one command unit per encoder count.
MOTOR_ENCODER_COUNTS_PER_REV = 1 << 20
COMMAND_UNITS_PER_MOTOR_RAD = MOTOR_ENCODER_COUNTS_PER_REV / tau


def setup(context):
    keys = ["master_index", "alias", "position", "product_code", "position_units_per_rad",
            "velocity_units_per_rad_s", "max_velocity_rad_s", "max_travel_rad",
            "sync0_shift_ns", "commissioning_disabled"]
    values = {key: LaunchConfiguration(key).perform(context) for key in keys}
    parameters = "".join(f'<param name="{key}">{escape(value)}</param>' for key, value in values.items())
    description = f'''<robot name="bottom_motor_bench">
      <link name="base"/><link name="shaft"/>
      <joint name="bottom_motor" type="continuous"><parent link="base"/><child link="shaft"/>
        <axis xyz="0 0 1"/></joint>
      <ros2_control name="BottomMotor" type="system"><hardware>
        <plugin>riser_hardware/BottomCsvSystem</plugin>{parameters}
        <param name="cycle_ns">1000000</param></hardware>
        <joint name="bottom_motor"><command_interface name="velocity"/>
          <state_interface name="position"/><state_interface name="velocity"/>
          <state_interface name="ready"/></joint>
      </ros2_control></robot>'''
    config = LaunchConfiguration("controllers_file").perform(context)
    publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(description, value_type=str)}],
        output="screen",
    )
    manager = Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[config], remappings=[("robot_description", "/robot_description")],
        output="screen", on_exit=[Shutdown(reason="BottomMotor controller manager exited")],
    )
    nodes = [publisher, manager]
    if values["commissioning_disabled"] == "0":
        nodes.append(Node(
            package="controller_manager", executable="spawner",
            arguments=["motor_profile", "--inactive", "--param-file", config],
            output="screen",
        ))
    return nodes


def generate_launch_description():
    args = [DeclareLaunchArgument(key, default_value="0")
            for key in ["master_index", "alias", "position", "sync0_shift_ns"]]
    args += [DeclareLaunchArgument("commissioning_disabled", default_value="1",
                                   choices=["0", "1"])]
    args += [DeclareLaunchArgument(key, default_value=str(COMMAND_UNITS_PER_MOTOR_RAD))
             for key in ["position_units_per_rad", "velocity_units_per_rad_s"]]
    args += [DeclareLaunchArgument("product_code", default_value="0x00000004")]
    args += [DeclareLaunchArgument(key) for key in
             ["max_velocity_rad_s", "max_travel_rad"]]
    args += [DeclareLaunchArgument("controllers_file", default_value=str(Path(
        get_package_share_directory("riser_motion_bringup")) / "config/controllers.yaml"))]
    return LaunchDescription(args + [OpaqueFunction(function=setup)])
