#pragma once

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace riser_kinematics {

inline constexpr std::size_t AngularCount = 3;
inline constexpr std::size_t ActuatorCount = 4;
inline constexpr std::array<const char *, AngularCount + ActuatorCount>
    CommandJointNames{"backboard_pitch", "foot_plate_pitch", "ml_angle",
                      "top_actuator",    "torso_actuator",   "hip_actuator",
                      "bottom_actuator"};

enum class CommandMode {
  AngularPosition,
  AngularVelocity,
  ActuatorPosition,
  ActuatorVelocity,
  None
};

inline bool angular(CommandMode mode) noexcept {
  return mode == CommandMode::AngularPosition ||
         mode == CommandMode::AngularVelocity;
}
inline bool velocity(CommandMode mode) noexcept {
  return mode == CommandMode::AngularVelocity ||
         mode == CommandMode::ActuatorVelocity;
}

// Each family owns all of its coordinates. The four masks are disjoint even
// though all four families drive the same physical mechanism.
inline std::optional<CommandMode>
command_family(const std::vector<std::string> &keys) {
  constexpr auto joint_count = CommandJointNames.size();
  static_assert(2 * joint_count <= 16);
  unsigned mask = 0;
  for (const std::string_view key : keys) {
    const auto slash = key.rfind('/');
    if (slash == std::string_view::npos)
      continue;
    const auto type = key.substr(slash + 1);
    if (type != "position" && type != "velocity")
      continue;
    for (std::size_t i = 0; i < joint_count; ++i) {
      if (key.substr(0, slash) != CommandJointNames[i])
        continue;
      const unsigned bit = 1u << (i + (type == "velocity" ? joint_count : 0));
      if (mask & bit)
        return std::nullopt;
      mask |= bit;
      break;
    }
  }
  constexpr unsigned angles = (1u << AngularCount) - 1;
  constexpr unsigned actuators = ((1u << ActuatorCount) - 1) << AngularCount;
  switch (mask) {
  case 0:
    return CommandMode::None;
  case angles:
    return CommandMode::AngularPosition;
  case angles << joint_count:
    return CommandMode::AngularVelocity;
  case actuators:
    return CommandMode::ActuatorPosition;
  case actuators << joint_count:
    return CommandMode::ActuatorVelocity;
  default:
    return std::nullopt;
  }
}

struct CommandModeSwitch {
  CommandMode mode;
  bool reseed;
};

// ros2_control passes global lists. Fold this system's changes into its
// current mode and leave it alone when the request touches only other devices.
inline std::optional<CommandModeSwitch>
plan_command_mode_switch(CommandMode current,
                         const std::vector<std::string> &start,
                         const std::vector<std::string> &stop) {
  const auto incoming = command_family(start);
  const auto outgoing = command_family(stop);
  if (!incoming || !outgoing)
    return std::nullopt;
  const bool starts = *incoming != CommandMode::None;
  const bool stops = *outgoing != CommandMode::None;
  if ((stops && *outgoing != current) ||
      (starts && current != CommandMode::None && !stops))
    return std::nullopt;
  return CommandModeSwitch{starts || stops ? *incoming : current,
                           starts || stops};
}

} // namespace riser_kinematics
