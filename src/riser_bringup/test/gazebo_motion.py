"""Exercise the motion workflow against a running, isolated Gazebo launch."""

import argparse
import time

import rclpy
from ament_index_python.packages import get_package_share_path
from builtin_interfaces.msg import Time
from control_msgs.msg import DynamicJointState
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from riser_motion.srv import Jog
from std_msgs.msg import String


class MotionCheck:
    def __init__(self):
        self.node = rclpy.create_node("gazebo_motion_check")
        self.description = ""
        self.feedback = {}
        self.sim_time = 0.
        self.clients = {}
        self.node.create_subscription(
            String, "/robot_description",
            lambda msg: setattr(self, "description", msg.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.node.create_subscription(
            DynamicJointState, "/g5_plot/dynamic_joint_states", self.read,
            qos_profile_sensor_data,
        )

    def read(self, msg):
        self.sim_time = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.feedback = {
            name: dict(zip(values.interface_names, values.values))
            for name, values in zip(msg.joint_names, msg.interface_values)
        }

    def wait(self, predicate, timeout, description):
        end = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= end:
                raise AssertionError(f"Timed out waiting for {description}: {self.feedback}")
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def call(self, service, path, request):
        if path not in self.clients:
            self.clients[path] = self.node.create_client(service, path)
        client = self.clients[path]
        assert client.wait_for_service(timeout_sec=10), path
        future = client.call_async(request)
        self.wait(future.done, 12, path)
        return future.result()

    def switch(self, start=(), stop=(), accepted=True):
        request = SwitchController.Request(
            activate_controllers=list(start), deactivate_controllers=list(stop),
            strictness=SwitchController.Request.STRICT, activate_asap=True,
        )
        request.timeout.sec = 5
        result = self.call(SwitchController, "/controller_manager/switch_controller", request)
        assert result.ok == accepted, (start, stop, result)
        print("Switch", start, stop, result.ok, flush=True)

    def jog(self, controller, **kwargs):
        request = Jog.Request(stamp=self.node.get_clock().now().to_msg(), **kwargs)
        result = self.call(Jog, f"/{controller}/jog", request)
        return request, result

    def move(self, controller, axis, *, sim_seconds, expect_position,
             cancel_after=None, lose_heartbeat=False, **kwargs):
        # Switching ownership reseeds the Gazebo servos from measured position.
        settled = time.monotonic() + .25
        while time.monotonic() < settled:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.wait(lambda: abs(self.feedback[axis]["velocity"]) < .0005, 3, "settled axis")
        start = self.feedback[axis]["position"]
        beats = self.node.create_publisher(Time, f"/{controller}/jog_heartbeat", 1)
        self.wait(lambda: beats.get_subscription_count() > 0, 5, "heartbeat subscription")
        request, result = self.jog(controller, **kwargs)
        assert result.accepted, result
        begun = time.monotonic()
        sim_start = self.sim_time
        cancelled = False
        displacement = 0.
        peak_rate = 0.
        endpoints = []
        while (elapsed := self.sim_time - sim_start) < sim_seconds:
            wall_elapsed = time.monotonic() - begun
            assert wall_elapsed < 4 * sim_seconds + 5, "Gazebo clock stalled"
            if not cancelled and (not lose_heartbeat or wall_elapsed < .35):
                beats.publish(request.stamp)
            if cancel_after and not cancelled and elapsed >= cancel_after:
                _, result = self.jog(controller, cancel=True)
                assert result.accepted, result
                cancelled = True
            rclpy.spin_once(self.node, timeout_sec=0.02)
            sample = self.feedback[axis]
            displacement = max(displacement, abs(sample["position"] - start))
            peak_rate = max(peak_rate, abs(sample["velocity"]))
            if elapsed >= sim_seconds - .15:
                endpoints.append(abs(sample["velocity"]))
        self.node.destroy_publisher(beats)
        print(f"{controller} {axis} moved {displacement:.6f}, "
              f"peak rate {peak_rate:.6f}, end rate {max(endpoints):.6f}", flush=True)
        assert displacement > expect_position, (controller, self.feedback[axis])
        assert peak_rate < .05 and max(endpoints) < .001, (controller, self.feedback[axis])
        return self.feedback[axis]["position"] - start

    def run(self, already_homed):
        self.wait(lambda: bool(self.description), 15, "robot description")
        assert "riser_gazebo/AngularSystem" in self.description
        assert "riser_hardware/BottomCsvSystem" not in self.description
        names = ("calibration_controller", "motor_profile", "motor_position_profile",
                 "angle_profile", "angle_position_profile", "inverted_pendulum")
        end = time.monotonic() + 45
        while time.monotonic() < end:
            result = self.call(ListControllers, "/controller_manager/list_controllers",
                               ListControllers.Request())
            if all(any(c.name == name and c.state == "inactive"
                       for c in result.controller) for name in names):
                break
            time.sleep(0.2)
        else:
            raise AssertionError(f"Controllers were not configured: {result}")
        motor, foot = "bottom_motor", "foot_plate_pitch"
        self.wait(lambda: motor in self.feedback and foot in self.feedback, 10, "feedback")
        if not already_homed:
            assert self.feedback[foot]["valid"] == 0
            self.switch(("angle_profile",), accepted=False)
            self.switch(("motor_profile",))
            self.move("motor_profile", motor, sim_seconds=1.8, expect_position=.001,
                      velocity_m_s=.005, duration_s=1.0)
            self.switch(stop=("motor_profile",))
            self.switch(("calibration_controller",))
            self.switch(("motor_profile",), accepted=False)
            self.wait(lambda: self.feedback[foot]["valid"] == 1, 360, "all midpoint references")
            print(f"Midpoint handoff feedback {self.feedback[foot]}", flush=True)
            self.switch(("angle_profile",), ("calibration_controller",))
        else:
            assert self.feedback[foot]["valid"] == 1
            self.switch(("angle_profile",))
        self.switch(("motor_profile",), accepted=False)
        for sign in (1, -1):
            motor_start = self.feedback[motor]["position"]
            motion = self.move("angle_profile", foot, sim_seconds=1.6,
                               expect_position=.001, velocity_m_s=sign * .005,
                               duration_s=1.0)
            assert sign * motion > .001, self.feedback[foot]
            assert sign * (self.feedback[motor]["position"] - motor_start) > .0003, self.feedback[motor]
        config = get_package_share_path("riser_motion") / "config"
        for mode, controller, axis in [
            ("velocity", "angle_profile", foot),
            ("velocity", "motor_profile", motor),
            ("position", "motor_position_profile", motor),
            ("position", "angle_position_profile", foot),
        ]:
            if controller != "angle_profile":
                self.switch((controller,))
            csv = (config / f"motion_{mode}.csv").read_text()
            _, result = self.jog(controller, csv=csv, preview=True)
            assert result.accepted, result
            self.move(controller, axis, sim_seconds=4, expect_position=.002, csv=csv)
            if controller == "motor_profile":
                rejected = (config / "motion_rejected.csv").read_text()
                for preview in (True, False):
                    _, result = self.jog(controller, csv=rejected, preview=preview)
                    assert not result.accepted and "increase" in result.message, result
                print("Nonmonotonic CSV rejected at preview and execution", flush=True)
                self.move(controller, axis, sim_seconds=2.7, expect_position=.001,
                          profile="triangle", ramp_s=.3, velocity_m_s=.008, duration_s=2.)
                self.move(controller, axis, sim_seconds=1.9, expect_position=.001,
                          cancel_after=.55, velocity_m_s=.008, duration_s=3.)
                self.move(controller, axis, sim_seconds=1.9, expect_position=.0004,
                          lose_heartbeat=True, velocity_m_s=.008, duration_s=3.)
            self.switch(stop=(controller,))
        print("Gazebo motion checks passed", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--already-homed", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    check = MotionCheck()
    try:
        check.run(args.already_homed)
    finally:
        check.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
