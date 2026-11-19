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

  CHECK(command_family(angular_position) == CommandMode::AngularPosition);
  CHECK(command_family(angular_velocity) == CommandMode::AngularVelocity);
  CHECK(command_family(actuator_position) == CommandMode::ActuatorPosition);
  CHECK(command_family(actuator_velocity) == CommandMode::ActuatorVelocity);

  auto change = plan_command_mode_switch(CommandMode::None, actuator_position, {});
  CHECK(change && change->mode == CommandMode::ActuatorPosition && change->reseed);
  change = plan_command_mode_switch(CommandMode::ActuatorPosition,
                                   angular_velocity, actuator_position);
  CHECK(change && change->mode == CommandMode::AngularVelocity && change->reseed);
  change = plan_command_mode_switch(CommandMode::AngularVelocity,
                                   angular_velocity, angular_velocity);
  CHECK(change && change->mode == CommandMode::AngularVelocity && change->reseed);
  change = plan_command_mode_switch(CommandMode::AngularVelocity, {}, angular_velocity);
  CHECK(change && change->mode == CommandMode::None && change->reseed);
  change = plan_command_mode_switch(CommandMode::AngularVelocity,
                                   {"another_system/velocity"}, {});
  CHECK(change && change->mode == CommandMode::AngularVelocity && !change->reseed);

  CHECK(!plan_command_mode_switch(CommandMode::ActuatorVelocity, angular_velocity, {}));
  CHECK(!plan_command_mode_switch(CommandMode::AngularPosition, {}, angular_velocity));
  CHECK(!plan_command_mode_switch(CommandMode::None, {angular_position[0]}, {}));
  auto mixed = actuator_velocity;
  mixed.push_back(angular_velocity[0]);
  CHECK(!plan_command_mode_switch(CommandMode::None, mixed, {}));
  mixed.back() = mixed.front();
  CHECK(!plan_command_mode_switch(CommandMode::None, mixed, {}));
}
