#!/usr/bin/env python3
"""Passive shaft calibration. The operator presents each stationary reference."""

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from actuator_mapping import load_maps
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import DynamicJointState, InterfaceValue
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger


@dataclass
class References:
    negative: float = math.nan
    zero: float = math.nan
    positive: float = math.nan

    @property
    def valid(self):
        return self.negative < self.zero < self.positive

    def capture(self, kind, sample):
        if (
            sample.get("feedback_valid") != 1
            or sample.get("ready") not in (1, 2)
            or sample.get("velocity") != 0
            or not math.isfinite(sample["position"])
        ):
            raise ValueError("Require stationary, healthy shaft feedback")
        limits = (sample.get("reverse_limit"), sample.get("forward_limit"))
        required = {"negative": (1, 0), "positive": (0, 1), "zero": (0, 0)}[kind]
        if limits != required:
            raise ValueError("Limit indications do not match reference")
        setattr(self, kind, sample["position"])


class Calibration(Node):
    def __init__(self):
        super().__init__("g5_calibration")
        share = Path(get_package_share_directory("riser_bringup"))
        self.model = json.loads((share / "urdf/model/kinematics.yaml").read_text())
        self.ml = json.loads((share / "config/control_coordinates.json").read_text())[
            "ml"
        ]
        self.maps = load_maps(
            Path(get_package_share_directory("riser_kinematics"))
            / "config/actuator_mapping.json"
        )
        self.axes = {}
        self.latest = {}
        self.last = 0.0
        self.writing = False
        self.add_on_set_parameters_callback(self.protect_results)
        self.pub = self.create_publisher(DynamicJointState, "~/state", 1)
        self.create_subscription(
            DynamicJointState,
            "/g5_plot/dynamic_joint_states",
            self.update,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self.watchdog)

    def protect_results(self, params):
        return SetParametersResult(
            successful=self.writing
            or all(not p.name.startswith("result.") for p in params),
            reason="Results are computed from observed references",
        )

    def announce(self, axis):
        ref = self.axes[axis]
        values = {
            "zero": ref.zero,
            "positive_travel": ref.positive - ref.zero if ref.valid else math.nan,
            "negative_travel": ref.zero - ref.negative if ref.valid else math.nan,
            "valid": ref.valid,
            "units": "rad",
        }
        self.writing = True
        try:
            for k, v in values.items():
                name = f"result.{axis}.{k}"
                if not self.has_parameter(name):
                    self.declare_parameter(name, v, ignore_override=True)
                else:
                    self.set_parameters([rclpy.parameter.Parameter(name, value=v)])
        finally:
            self.writing = False

    def watchdog(self):
        if self.latest and time.monotonic() - self.last > 0.3:
            self.latest.clear()
            for axis in self.axes:
                self.axes[axis] = References()
                self.announce(axis)
            self.publish()

    def capture(self, axis, kind, response):
        try:
            if time.monotonic() - self.last > 0.3:
                raise ValueError("Stale feedback")
            self.axes[axis].capture(kind, self.latest[axis])
            self.announce(axis)
            self.publish()
            response.success = True
            response.message = f"{axis} {kind} reference captured"
        except (ValueError, KeyError) as exc:
            response.message = str(exc)
        return response

    def update(self, msg):
        self.watchdog()
        age = self.get_clock().now().nanoseconds * 1e-9 - (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )
        if abs(age) > 0.3:
            self.watchdog()
            return
        self.last = time.monotonic() - abs(age)
        self.stamp = msg.header.stamp
        self.latest = {
            axis: dict(zip(v.interface_names, v.values))
            for axis, v in zip(msg.joint_names, msg.interface_values)
        }
        for axis in self.latest:
            if axis not in ("bottom_motor", "top_motor", "torso_motor", "hip_motor"):
                continue
            if axis not in self.axes:
                self.axes[axis] = References()
                self.announce(axis)
                self.declare_parameter(
                    axis + ".metres_per_rad", 0.0, ParameterDescriptor(read_only=True)
                )
                for kind in ("zero", "negative", "positive"):
                    self.create_service(
                        Trigger,
                        f"~/{axis}/capture_{kind}",
                        lambda req, res, a=axis, k=kind: self.capture(a, k, res),
                    )
        for axis, ref in self.axes.items():
            sample = self.latest.get(axis, {})
            if (
                sample.get("feedback_valid") != 1 or sample.get("ready") not in (1, 2)
            ) and any(math.isfinite(x) for x in (ref.zero, ref.positive, ref.negative)):
                self.axes[axis] = References()
                self.announce(axis)
        self.publish()

    def publish(self):
        result = DynamicJointState()
        result.header.stamp = self.stamp
        angles = {}
        for axis, ref in self.axes.items():
            result.joint_names.append(axis)
            result.interface_values.append(
                InterfaceValue(
                    interface_names=[
                        "zero_rad",
                        "negative_travel_rad",
                        "positive_travel_rad",
                        "valid",
                    ],
                    values=[
                        ref.zero,
                        ref.zero - ref.negative if ref.valid else math.nan,
                        ref.positive - ref.zero if ref.valid else math.nan,
                        float(ref.valid),
                    ],
                )
            )
            scale = self.get_parameter(axis + ".metres_per_rad").value
            if (
                not ref.valid
                or not math.isfinite(scale)
                or scale == 0
                or not ref.negative <= self.latest[axis]["position"] <= ref.positive
            ):
                continue
            key = {
                "bottom_motor": "foot_pitch",
                "top_motor": "frame_pitch",
                "torso_motor": "torso_lateral",
                "hip_motor": "hip_lateral",
            }[axis]
            neutral = self.model["neutral_cad"][key]
            domain = self.model["joints"][key]
            mapping = self.maps[key]
            s = (self.latest[axis]["position"] - ref.zero) * scale + mapping.forward(
                neutral
            )
            try:
                angles[key] = (
                    mapping.inverse(
                        s, domain["lower"] + neutral, domain["upper"] + neutral
                    )
                    - neutral
                )
            except ValueError:
                pass
        h = self.ml["height_m"]
        ml = math.atan(
            (
                h["torso"] * angles.get("torso_lateral", math.nan)
                + h["hip"] * angles.get("hip_lateral", math.nan)
            )
            / (h["torso"] ** 2 + h["hip"] ** 2)
        )
        result.joint_names.append("calibrated_angles")
        result.interface_values.append(
            InterfaceValue(
                interface_names=["footplate_rad", "backboard_rad", "ml_rad"],
                values=[
                    angles.get("foot_pitch", math.nan),
                    angles.get("frame_pitch", math.nan),
                    ml,
                ],
            )
        )
        self.pub.publish(result)


if __name__ == "__main__":
    rclpy.init()
    node = Calibration()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
