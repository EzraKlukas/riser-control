#include "riser_kinematics/command_mode.hpp"
#include <iostream>

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      std::cerr << "Failed at line " << __LINE__ << ": " << #condition << '\n'; \
      return 1;                                                                \
    }                                                                          \
  } while (false)

int main() {
  using namespace riser_kinematics;
  using Keys = std::vector<std::string>;
  const auto keys = [](bool use_angles, const std::string &type) {
    Keys result;
    for (std::size_t i = 0; i < CommandJointNames.size(); ++i)
      if ((i < AngularCount) == use_angles)
        result.push_back(CommandJointNames[i] + type);
    return result;
  };
  const Keys angular_position = keys(true, "/position");
  const Keys angular_velocity = keys(true, "/velocity");
  const Keys actuator_position = keys(false, "/position");
  const Keys actuator_velocity = keys(false, "/velocity");

  CHECK(command_claim(angular_position)->mode == CommandMode::AngularPosition);
  CHECK(command_claim(angular_velocity)->mode == CommandMode::AngularVelocity);
  CHECK(command_claim(actuator_position)->mode == CommandMode::ActuatorPosition);
  CHECK(command_claim(actuator_velocity)->mode == CommandMode::ActuatorVelocity);

  const Keys foot{"foot_plate_pitch/velocity"};
  const Keys bottom{"bottom_motor/velocity"};
  Keys calibration = actuator_velocity;
  for (const auto &key : actuator_velocity) {
    const auto motor = key.substr(0, key.find('/'));
    calibration.push_back(motor + "/calibration_midpoint");
    calibration.push_back(motor + "/calibration_half_travel");
  }
  const auto calibrating = plan_command_mode_switch({}, calibration, {});
  CHECK(calibrating && calibrating->claim.calibration && calibrating->reseed);
  CHECK(!plan_command_mode_switch(calibrating->claim, foot, {}));
  CHECK(!plan_command_mode_switch(calibrating->claim, {}, bottom));
  const auto handoff = plan_command_mode_switch(calibrating->claim, foot, calibration);
  CHECK(handoff && handoff->claim.mode == CommandMode::AngularVelocity);
  CHECK(!angles_ready(handoff->claim, 0));
  CHECK(angles_ready(handoff->claim, 1u << 1));
  const Keys bottom_calibration{"bottom_motor/velocity", "bottom_motor/calibration_midpoint",
                               "bottom_motor/calibration_half_travel"};
  const auto bottom_run = plan_command_mode_switch({}, bottom_calibration, {});
  CHECK(bottom_run);
  const auto bottom_handoff = plan_command_mode_switch(bottom_run->claim, foot, bottom_calibration);
  CHECK(bottom_handoff && angles_ready(bottom_handoff->claim, 1u << 1));
  const auto outside = plan_command_mode_switch(handoff->claim, {"unrelated/velocity"}, {});
  CHECK(outside && outside->claim == handoff->claim && !outside->reseed);
  const auto idle = plan_command_mode_switch(handoff->claim, {}, foot);
  CHECK(idle && idle->claim.mode == CommandMode::None && idle->reseed);
  CHECK(!plan_command_mode_switch({}, {"bottom_motor/calibration_midpoint"}, {}));
  auto mixed = actuator_velocity;
  mixed.push_back(angular_velocity[0]);
  CHECK(!plan_command_mode_switch({}, mixed, {}));
  mixed.back() = mixed.front();
  CHECK(!plan_command_mode_switch({}, mixed, {}));
}
