#!/usr/bin/env python3
"""Stamp-preserving adapter for Fortress's scoped sensor frame names.
No geometric transform is performed: sensor/joint/link origins and axes are
identical in generated SDF. Both are at the configured reporting origin.
The node does not read its own clock: every output preserves the sensor timestamp.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import try_shutdown
from geometry_msgs.msg import WrenchStamped


class PlateFrames(Node):
    def __init__(self):
        super().__init__("force_plate_frames")
        self.subscriptions_ = []
        for side in ("left", "right"):
            pub = self.create_publisher(
                WrenchStamped, "/riser/force_plates/" + side + "/wrench", 100
            )

            def receive(msg, pub=pub, frame=side + "_force_plate"):
                msg.header.frame_id = frame
                pub.publish(msg)

            self.subscriptions_.append(
                self.create_subscription(
                    WrenchStamped, "/riser/force_plates/" + side + "/raw", receive, 100
                )
            )


def main():
    rclpy.init()
    node = PlateFrames()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble may interrupt take_message while the context is shutting down.
        # Preserve errors during normal operation; suppress only shutdown races.
        if node.context.ok():
            raise
    finally:
        node.destroy_node()
        try_shutdown(context=node.context)


if __name__ == "__main__":
    main()
