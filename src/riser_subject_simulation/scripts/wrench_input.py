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

        self.pub = self.create_publisher(EntityWrench, "/subject_simulation/wrench_input", 10)
        self.create_timer(.05, self.send)

    def send(self):
        msg_l, msg_r = copy.deepcopy(self.left), copy.deepcopy(self.right)
        self.pub.publish(msg_l)
        self.pub.publish(msg_r)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true", help="Open the wrench control window")
    args, ros_args = parser.parse_known_args()
    if args.gui:
        from PyQt5.QtCore import QTimer
        from PyQt5.QtWidgets import (
            QApplication, QDoubleSpinBox, QGridLayout, QLabel, QPushButton, QWidget,
        )
        from rclpy.executors import ExternalShutdownException

    rclpy.init(args=ros_args)
    node = WrenchInput()
    try:
        if not args.gui:
            print("doesn't see gui parameter!")
            rclpy.spin(node)
            return

        app = QApplication([])
        window = QWidget()
        window.setWindowTitle("World-frame wrench inputs")
        grid = QGridLayout(window)
        controls = []
        for column, (title, message) in enumerate(
            (("Left plate", node.left), ("Right plate", node.right)), 1
        ):
            grid.addWidget(QLabel(title), 0, column)
            for group, (kind, unit) in enumerate((("force", "N"), ("torque", "N·m"))):
                vector = getattr(message.wrench, kind)
                for index, axis in enumerate("xyz"):
                    row = 1 + group * 3 + index
                    if column == 1:
                        grid.addWidget(QLabel(f"{kind.title()} {axis} ({unit})"), row, 0)
                    box = QDoubleSpinBox()
                    box.setRange(-10000.0, 10000.0)
                    box.setDecimals(2)
                    box.setSingleStep(1.0)
                    box.setKeyboardTracking(False)
                    box.setValue(getattr(vector, axis))
                    box.valueChanged.connect(
                        lambda value, vector=vector, axis=axis: setattr(vector, axis, value)
                    )
                    grid.addWidget(box, row, column)
                    controls.append(box)

        def zero_all():
            for box in controls:
                box.setValue(0.0)

        zero = QPushButton("Zero all")
        zero.clicked.connect(zero_all)
        grid.addWidget(zero, 7, 0, 1, 3)

        def poll():
            if not rclpy.ok():
                app.quit()
                return
            try:
                rclpy.spin_once(node, timeout_sec=0)
            except ExternalShutdownException:
                app.quit()

        timer = QTimer(window)
        timer.timeout.connect(poll)
        timer.start(25)  # 40 Hz polling; the node publishes at 20 Hz.
        window.show()
        app.exec_()
        timer.stop()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
