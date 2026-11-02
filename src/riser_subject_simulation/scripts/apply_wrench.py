#!/usr/bin/env python3

from ros_gz_interfaces.msg import Entity, EntityWrench
from rclpy.node import Node

class WrenchInput(Node):
    def __init__(self):
        super().__init__("simulation_wrench_input")
        # should probably represent as two from the start, since only one force plate can sense what we're doing at a time.
        self.left = EntityWrench;
        self.right = EntityWrench;
        self.pub = self.create_publisher(EntityWrench, "/subject_simulation/force_input", 10)
