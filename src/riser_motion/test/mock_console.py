"""Run manually with Jazzy and PySide6. Uses a separate localhost mock graph."""

import os
import runpy
import signal
import subprocess
import tempfile
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix


def check(axes):
    os.environ.update(
        ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", QT_QPA_PLATFORM="offscreen"
    )
    if "ROS_DOMAIN_ID" not in os.environ:
        raise ValueError("Set ROS_DOMAIN_ID to an unused test domain")
    import rclpy
    from PySide6.QtWidgets import QApplication, QTabWidget

    console = runpy.run_path(
        str(
            Path(get_package_prefix("riser_motion"))
            / "lib/riser_motion/g5_console"
        )
    )["Console"]
    with tempfile.TemporaryDirectory() as directory:
        os.environ["ROS_LOG_DIR"] = directory
        app = QApplication([])
        rclpy.init()
        window = console("unused-mock-host")
        window.show()

        def until(predicate):
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                app.processEvents()
                if predicate():
                    return
                time.sleep(0.003)
            raise AssertionError(
                window.graph.toPlainText()
                + "\n"
                + window.readout.toPlainText()
                + "\n"
                + window.output.toPlainText()
            )

        with (Path(directory) / "launch.log").open("w+") as log:
            launch = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "riser_motion",
                    "bottom_motor.launch.py",
                    "drive_backend:=mock",
                    "enabled_axes:=" + axes,
                    "commissioning_disabled:=0",
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                until(
                    lambda: (
                        len(window.samples) == len(axes.split(","))
                        and "motor_profile  inactive" in window.graph.toPlainText()
                        and "calibration_controller  inactive"
                        in window.graph.toPlainText()
                    )
                )
                assert set(window.excluded) == {"bottom", "top", "torso", "hip"} - set(
                    axes.split(",")
                )
                assert float(window.hw["position_units_per_m"]) == 104_857_600
                window.readout.setFixedHeight(100)
                scrollbar = window.readout.verticalScrollBar()
                until(lambda: scrollbar.maximum() > 2)
                scrollbar.setValue(2)
                window.readout.setPlainText("previous sample\n" * 30)
                scrollbar.setValue(2)
                window.tick()
                assert scrollbar.value() == 2
                window.findChild(QTabWidget).setCurrentIndex(2)
                window.graph.setFixedHeight(100)
                scrollbar = window.graph.verticalScrollBar()
                until(lambda: scrollbar.maximum() > 2)
                previous_graph = "previous snapshot\n" * 30
                window.graph.setPlainText(previous_graph)
                scrollbar.setValue(2)
                window.poll()
                assert window.graph.toPlainText() != previous_graph
                assert scrollbar.value() == 2
                for state in ("active", "inactive"):
                    subprocess.run(
                        [
                            "ros2",
                            "control",
                            "set_controller_state",
                            "motor_profile",
                            state,
                        ],
                        check=True,
                        timeout=10,
                    )
                    until(
                        lambda state=state: (
                            "motor_profile  " + state in window.graph.toPlainText()
                        )
                    )
                    if state == "active":
                        until(
                            lambda: (
                                "bottom_motor/velocity  available=True claimed=True"
                                in window.graph.toPlainText()
                            )
                        )
                        window.axis.setCurrentText("bottom_motor")
                        # Discover the service before sending a stamped request.
                        from riser_motion.srv import Jog

                        client = window.node.create_client(Jog, "/motor_profile/jog")
                        window.clients["motor_profile"] = client
                        until(client.service_is_ready)
                        until(lambda: "motor_profile" in window.caps)
                        for key in ("max_velocity_m_s", "max_travel_m"):
                            assert window.caps["motor_profile"][key] == float(
                                window.hw[key]
                            )
                        window.jog(False)
                        until(lambda: "motor_profile" in window.pulses)
                        until(lambda: window.samples["bottom_motor"]["velocity"] > 0)
                        window.jog(True)
                        until(lambda: window.samples["bottom_motor"]["velocity"] == 0)
                        window.speed.setText("-0.01")
                        window.jog(False)
                        until(lambda: window.samples["bottom_motor"]["velocity"] < 0)
                        window.pulses.clear()
                        until(lambda: window.samples["bottom_motor"]["velocity"] == 0)
                window.param.setText("max_duration_s")
                subprocess.run(
                    ["ros2", "param", "set", "/motor_profile", "max_duration_s", "7.0"],
                    check=True,
                    timeout=10,
                )
                until(
                    lambda: "max_duration_s  (float)  7.0" in window.graph.toPlainText()
                )
                until(
                    lambda: (
                        window.caps.get("motor_profile", {}).get("max_duration_s")
                        == 7.0
                    )
                )
                print(
                    axes,
                    "GUI, CLI parameter and claim updates, signed jog, cancel and heartbeat expiry passed",
                )
            except Exception:
                log.seek(0)
                print(log.read()[-3500:])
                raise
            finally:
                os.kill(launch.pid, signal.SIGINT)
                try:
                    launch.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGTERM)
                    launch.wait(timeout=5)
                window.dispose()
                rclpy.shutdown()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--axes", default="bottom,top,torso,hip")
    check(parser.parse_args().axes)
