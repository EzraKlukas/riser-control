"""Passive operator capture and loss of the encoder session."""

import math
from pathlib import Path
import sys
import time

import rclpy
from control_msgs.msg import DynamicJointState, InterfaceValue
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from g5_calibration import Calibration


def test_bottom_only_references():
    rclpy.init(args=[])
    node = Calibration()
    output = []
    node.pub.publish = output.append

    def feed(position, neg=0.0, pos=0.0, healthy=1.0):
        msg = DynamicJointState(
            joint_names=["bottom_motor"],
            interface_values=[
                InterfaceValue(
                    interface_names=[
                        "position",
                        "velocity",
                        "reverse_limit",
                        "forward_limit",
                        "ready",
                        "feedback_valid",
                    ],
                    values=[position, 0.0, neg, pos, 2.0, healthy],
                )
            ],
        )
        msg.header.stamp = node.get_clock().now().to_msg()
        node.update(msg)

    def capture(kind):
        return node.capture("bottom_motor", kind, Trigger.Response()).success

    try:
        feed(-0.002, 1.0, 1.0)
        assert not capture("negative")
        feed(-0.002, 1.0)
        assert capture("negative")
        feed(0.004, 0.0, 1.0)
        assert capture("positive")
        assert not node.axes["bottom_motor"].valid
        feed(0.0)
        assert capture("zero")
        for key, value in {
            "zero": 0.0,
            "negative_travel": 0.002,
            "positive_travel": 0.004,
            "units": "m",
            "valid": True,
        }.items():
            assert node.get_parameter("result.bottom_motor." + key).value == value
        angles = output[-1].interface_values[-1].values
        assert abs(angles[0]) < 1e-8 and math.isnan(angles[1]) and math.isnan(angles[2])
        feed(0.001)
        assert abs(output[-1].interface_values[-1].values[0]) > 1e-5
        node.last = time.monotonic() - 1
        assert not capture("zero")
        node.watchdog()
        assert not node.get_parameter("result.bottom_motor.valid").value
        assert math.isnan(output[-1].interface_values[-1].values[0])
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_four_axis_angles_invalidate_only_missing_inputs():
    axes = ("bottom_motor", "top_motor", "torso_motor", "hip_motor")
    rclpy.init(args=[])
    node = Calibration()
    output = []
    node.pub.publish = output.append

    def feed(position, negative=0.0, positive=0.0, invalid=""):
        message = DynamicJointState()
        message.header.stamp = node.get_clock().now().to_msg()
        for axis in axes:
            message.joint_names.append(axis)
            message.interface_values.append(
                InterfaceValue(
                    interface_names=[
                        "position",
                        "velocity",
                        "ready",
                        "feedback_valid",
                        "reverse_limit",
                        "forward_limit",
                    ],
                    values=[
                        position,
                        0.0,
                        1.0,
                        float(axis != invalid),
                        negative,
                        positive,
                    ],
                )
            )
        node.update(message)

    try:
        for reference, sample in [
            ("negative", (-0.002, 1.0, 0.0)),
            ("positive", (0.004, 0.0, 1.0)),
            ("zero", (0.0,)),
        ]:
            feed(*sample)
            for axis in axes:
                assert node.capture(axis, reference, Trigger.Response()).success
        feed(0.0003)
        pitch, backboard, ml = output[-1].interface_values[-1].values
        assert all(math.isfinite(angle) for angle in (pitch, backboard, ml))
        feed(0.0003, invalid="hip_motor")
        assert not node.get_parameter("result.hip_motor.valid").value
        next_pitch, next_backboard, next_ml = output[-1].interface_values[-1].values
        assert (next_pitch, next_backboard) == (pitch, backboard)
        assert math.isnan(next_ml)
    finally:
        node.destroy_node()
        rclpy.shutdown()
