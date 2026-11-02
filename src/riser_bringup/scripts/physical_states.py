#!/usr/bin/env python3
"""Kinematic TF states for mock/RViz; Gazebo publishes actual physical states."""

import json
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import try_shutdown
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory


class PhysicalStates(Node):
    def __init__(self):
        super().__init__("riser_physical_states")
        p = Path(get_package_share_directory("riser_bringup"))
        self.coordinates = json.loads(
            (p / "config/control_coordinates.json").read_text()
        )
        self.publisher = self.create_publisher(
            JointState, "/riser/physical_joint_states", 10
        )
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.update, 10
        )

    def update(self, msg):
        q = dict(zip(msg.name, msg.position))
        if any(k not in q for k in self.coordinates["public_joints"]):
            return
        out = JointState()
        out.header = msg.header
        values = {n: q[n] for n in self.coordinates["public_joints"]}
        for s in ["torso", "hip"]:
            values[s + "_lateral"] = self.coordinates["ml"]["height_m"][s] * math.tan(
                q["ml_angle"]
            )
        out.name = list(values)
        out.position = list(values.values())
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = PhysicalStates()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if node.context.ok():
            raise
    finally:
        node.destroy_node()
        try_shutdown(context=node.context)


if __name__ == "__main__":
    main()
