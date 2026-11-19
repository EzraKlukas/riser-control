#!/usr/bin/env python3
"""Give the 25 Hz ankle plot data named, stamped wrench fields."""

from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import WrenchStamped
import rclpy
from rclpy.node import Node


class AnklePlotWrenches(Node):
    def __init__(self):
        super().__init__("ankle_plot_wrenches")
        self.before = self.create_publisher(
            WrenchStamped, "/riser/plot/ankle_before_perturbation", 1
        )
        self.exported = self.create_publisher(
            WrenchStamped, "/riser/plot/ankle_exported", 1
        )
        self.create_subscription(
            DynamicJointState, "/ankle_wrench_plot_broadcaster/dynamic_joint_states",
            self.on_state, 1,
        )

    def on_state(self, message):
        if "ankle_wrench" not in message.joint_names:
            return
        entry = message.interface_values[message.joint_names.index("ankle_wrench")]
        values = dict(zip(entry.interface_names, entry.values))
        wrench = WrenchStamped()
        wrench.header = message.header
        try:
            wrench.wrench.force.x = values["force.x"]
            wrench.wrench.force.y = values["force.y"]
            wrench.wrench.force.z = values["force.z"]
            wrench.wrench.torque.x = values["torque.x"]
            wrench.wrench.torque.y = values["torque.y"]
            wrench.wrench.torque.z = values["torque.z"]
            before_x = values["torque.x_before_perturbation"]
        except KeyError:
            return

        self.exported.publish(wrench)
        wrench.wrench.torque.x = before_x
        self.before.publish(wrench)


def main():
    rclpy.init()
    node = AnklePlotWrenches()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
