"""ROS-installed Xacro acceptance checks for the angular interface boundary."""

from pathlib import Path
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
                assert [j.get("name") for j in control.findall("joint")] == [
                    "backboard_pitch",
                    "foot_plate_pitch",
                    "ml_angle",
                ]
                for j in control.findall("joint"):
                    assert [x.get("name") for x in j.findall("command_interface")] == [
                        mode
                    ]
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
        assert root.find("link[@name='%s']/collision" % name) is not None


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


@pytest.mark.parametrize("backend", ["mock", "gazebo", "ethercat"])
@pytest.mark.parametrize("mode", ["effort", "acceleration"])
def test_unsupported_command_is_rejected(backend, mode):
    share = Path(get_package_share_directory("riser_bringup"))
    with pytest.raises(xacro.XacroException, match=mode):
        xacro.process_file(
            str(share / "urdf/riser.urdf.xacro"),
            mappings={"backend": backend, "command_mode": mode},
        )
