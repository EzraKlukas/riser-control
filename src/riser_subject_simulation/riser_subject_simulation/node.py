#!/usr/bin/env python3

import argparse
import copy
import rclpy
from ros_gz_interfaces.msg import Entity, EntityWrench
from rclpy.node import Node

class WrenchInput(Node):
    def __init__(self):
        super().__init__("simulation_wrench_input")
        self.left = EntityWrench();
        self.right = EntityWrench();

        self.left.entity.name = "riser::left_force_plate"
        self.right.entity.name = "riser::right_force_plate"

        # initial values to show that it's doing something.
        for side in [self.left, self.right]:
            side.entity.type = Entity.LINK
            side.wrench.force.x, side.wrench.force.y, side.wrench.force.z = 1.0, 1.0, 1.0
            side.wrench.torque.x, side.wrench.torque.y, side.wrench.torque.z = 1.0, 1.0, 1.0

        self.pub = self.create_publisher((EntityWrench, EntityWrench), "/subject_simulation/wrench_inputs", 10)
        self.create_timer(.05, self.send)

    def send(self):
        msg_l, msg_r = copy.deepcopy(self.left), copy.deepcopy(self.right)
        self.pub.publish((msg_l, msg_r))

def main():
    parser = argparse.ArgumentParser() # currently don't even care actually.
    parser.add_argument("--tui", action="store_true") # true if the argument is given, false if not.
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)

    node = WrenchInput()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
