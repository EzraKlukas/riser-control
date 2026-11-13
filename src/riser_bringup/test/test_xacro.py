"""ROS-installed Xacro acceptance checks for the angular interface boundary."""

from pathlib import Path
import json
import xml.etree.ElementTree as E
from ament_index_python.packages import get_package_share_directory
import xacro
import pytest


def test_angular_interfaces_and_shared_geometry():
    share = Path(get_package_share_directory("riser_bringup"))
    signatures = []
    plugins = {
        "none": None,
        "mock": "mock_components/GenericSystem",
        "gazebo": "riser_gazebo/AngularSystem",
        "ethercat": "riser_hardware/EthercatBoundary",
    }
    for backend, plugin in plugins.items():
        for mode in ["position", "velocity"]:
            root = E.fromstring(
                xacro.process_file(
                    str(share / "urdf/riser.urdf.xacro"),
                    mappings={"backend": backend, "command_mode": mode},
                ).toxml()
            )
            control = root.find("ros2_control")
            if plugin is None:
                assert control is None
            else:
                assert control.find("hardware/plugin").text == plugin
                expected = ["backboard_pitch", "foot_plate_pitch", "ml_angle"]
                assert [j.get("name") for j in control.findall("joint")] == expected
                expected_gpios = ["top_actuator", "torso_actuator", "hip_actuator", "bottom_actuator"] if backend == "gazebo" else []
                assert [g.get("name") for g in control.findall("gpio")] == expected_gpios
                expected_sensors = ["left_force_plate", "right_force_plate"] if backend == "gazebo" else []
                assert [s.get("name") for s in control.findall("sensor")] == expected_sensors
                for name in expected_gpios:
                    assert root.find(f"joint[@name='{name}']") is None
                for j in control.findall("joint") + control.findall("gpio"):
                    commands = ["position", "velocity"] if backend == "gazebo" else [mode]
                    assert [x.get("name") for x in j.findall("command_interface")] == commands
                    for parameter in ("lower", "upper", "velocity_limit"):
                        assert j.find(f"param[@name='{parameter}']") is not None
                    assert [x.get("name") for x in j.findall("state_interface")] == [
                        "position",
                        "velocity",
                        "effort",
                    ]
            signatures.append([E.tostring(j) for j in root.findall("joint")])
    assert all(s == signatures[0] for s in signatures)


def test_independent_sensing_paths_and_contact_geometry():
    share = Path(get_package_share_directory("riser_bringup"))
    root = E.fromstring(
        xacro.process_file(
            str(share / "urdf/riser.urdf.xacro"), mappings={"backend": "gazebo"}
        ).toxml()
    )
    plugin = root.find("gazebo/plugin")
    assert plugin.get("filename") == "libriser_gazebo_system.so"
    assert plugin.get("name") == "riser_gazebo::PhysicsClockControlPlugin"
    assert len(root.findall(".//visual")) == 11
    assert len(root.findall(".//collision")) == 4
    assert root.find("link[@name='foot_platform']/collision") is None
    for side in ["left", "right"]:
        name = side + "_force_plate"
        j = root.find("joint[@name='%s_sensor_joint']" % name)
        assert j.get("type") == "fixed"
        assert j.find("parent").get("link") == "foot_platform"
        assert j.find("child").get("link") == name
        gz = root.find("gazebo[@reference='%s_sensor_joint']" % name)
        assert gz.find("preserveFixedJoint").text == "true"
        assert gz.find("sensor/pose").text == "0 0 0 0 0 0"
        assert gz.find("sensor/force_torque/frame").text == "sensor"
        assert float(gz.find("sensor/update_rate").text) == 1000.
        assert root.find("link[@name='%s']/collision" % name) is not None
        sensor = root.find(f"ros2_control/sensor[@name='{name}']")
        assert sensor.find("command_interface") is None
        assert [s.get("name") for s in sensor.findall("state_interface")] == [
            "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z",
            "sample_time", "valid",
        ]
        assert sensor.find("param[@name='gz_topic']").text == gz.find("sensor/topic").text
        assert float(sensor.find("param[@name='sample_timeout']").text) > 0


def test_direct_pitch_model():
    share = Path(get_package_share_directory("riser_bringup"))
    for mode in ["position", "velocity"]:
        direct = E.fromstring(
            xacro.process_file(
                str(share / "urdf/riser.urdf.xacro"),
                mappings={"backend": "gazebo", "command_mode": mode},
            ).toxml()
        )
        assert len(direct.findall("link")) == 11
        assert direct.find("joint[@name='top_rod_hinge']") is None
        assert direct.find("joint[@name='bottom_rod_hinge']") is None
        assert direct.find(".//hardware/param[@name='pitch_drive']") is None


def test_neutral_frames_and_geometric_limits():
    share = Path(get_package_share_directory("riser_bringup"))
    k = json.loads((share / 'urdf/model/kinematics.yaml').read_text())
    c = json.loads((share / 'config/control_coordinates.json').read_text())
    root = E.fromstring(xacro.process_file(str(share / 'urdf/riser.urdf.xacro'),
        mappings={'backend': 'gazebo'}).toxml())
    hardware = root.find('ros2_control/hardware')
    axes = ['frame_pitch', 'torso_lateral', 'hip_lateral', 'foot_pitch']
    assert list(map(float, hardware.find("param[@name='neutral_cad']").text.split())) == [
        k['neutral_cad'][axis] for axis in axes]
    backboard = root.find("joint[@name='backboard_pitch']")
    assert list(map(float, backboard.find('origin').get('rpy').split())) == [0, 0, 0]
    assert float(backboard.find('limit').get('upper')) == k['joints']['frame_pitch']['upper']
    assert float(root.find(
        "ros2_control/joint[@name='backboard_pitch']/param[@name='upper']").text) == k['joints']['frame_pitch']['upper']
    for name in ('torso', 'hip'):
        joint = root.find(f"joint[@name='{name}_lateral']")
        y = float(joint.find('origin').get('xyz').split()[1])
        contact = root.find(f"link[@name='{name}_carriage']/collision/origin")
        assert abs(y + float(contact.get('xyz').split()[1])) < 1e-12
        assert float(joint.find('limit').get('upper')) == .335
    ml = root.find("joint[@name='ml_angle']/limit")
    assert float(ml.get('upper')) == c['ml']['limit_rad']
    assert float(ml.get('velocity')) == c['ml']['velocity_rad_s']


@pytest.mark.parametrize("backend", ["mock", "gazebo", "ethercat"])
@pytest.mark.parametrize("mode", ["effort", "acceleration"])
def test_unsupported_command_is_rejected(backend, mode):
    share = Path(get_package_share_directory("riser_bringup"))
    with pytest.raises(xacro.XacroException, match=mode):
        xacro.process_file(
            str(share / "urdf/riser.urdf.xacro"),
            mappings={"backend": backend, "command_mode": mode},
        )


@pytest.mark.parametrize("mode", ["angular_position", "angular_velocity", "actuator_position", "actuator_velocity"])
def test_gazebo_startup_modes(mode):
    share = Path(get_package_share_directory("riser_bringup"))
    root = E.fromstring(xacro.process_file(str(share / "urdf/riser.urdf.xacro"),
        mappings={"backend": "gazebo", "command_mode": mode}).toxml())
    assert root.find("ros2_control/hardware/param[@name='command_mode']").text == mode
    assert len(root.findall("ros2_control/joint")) == 3
    assert len(root.findall("ros2_control/gpio")) == 4
    resources = root.findall("ros2_control/joint") + root.findall("ros2_control/gpio")
    assert sum(len(r.findall("command_interface")) for r in resources) == 14
    k = json.loads((share / "urdf/model/kinematics.yaml").read_text())["joints"]
    for gpio in root.findall("ros2_control/gpio"):
        limits = k[gpio.get("name")]
        for parameter, key in [("lower", "lower"), ("upper", "upper"), ("velocity_limit", "velocity")]:
            assert float(gpio.find(f"param[@name='{parameter}']").text) == limits[key]
