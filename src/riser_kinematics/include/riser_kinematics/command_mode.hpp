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
                      "top_motor",       "torso_motor",      "hip_motor",
                      "bottom_motor"};

enum class CommandMode : unsigned char {
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

struct CommandClaim {
  CommandMode mode = CommandMode::None;
  unsigned char joints = 0;
  unsigned char calibration = 0;
  unsigned char reserved = 0;

  bool operator==(const CommandClaim &other) const {
    return mode == other.mode && joints == other.joints &&
           calibration == other.calibration;
  }
};

// Accept a subset of one family. The mechanism keeps one owner until its
// exact claim is released. Calibration metadata must accompany velocity.
inline std::optional<CommandClaim>
command_claim(const std::vector<std::string> &keys) {
  CommandClaim claim;
  unsigned midpoints = 0, spans = 0;
  for (const std::string_view key : keys) {
    const auto slash = key.rfind('/');
    if (slash == std::string_view::npos)
      continue;
    const auto type = key.substr(slash + 1);
    for (std::size_t i = 0; i < CommandJointNames.size(); ++i) {
      if (key.substr(0, slash) != CommandJointNames[i])
        continue;
      const bool angle = i < AngularCount;
      const unsigned bit = 1u << i;
      if (!angle && type == "calibration_midpoint") {
        midpoints |= bit;
      } else if (!angle && type == "calibration_half_travel") {
        spans |= bit;
      } else if (type == "position" || type == "velocity") {
        const auto mode =
            angle ? (type == "position" ? CommandMode::AngularPosition
                                        : CommandMode::AngularVelocity)
                  : (type == "position" ? CommandMode::ActuatorPosition
                                        : CommandMode::ActuatorVelocity);
        if ((claim.joints && claim.mode != mode) || (claim.joints & bit))
          return std::nullopt;
        claim.mode = mode;
        claim.joints |= bit;
      }
      break;
    }
  }
  if (midpoints != spans ||
      (midpoints && (claim.mode != CommandMode::ActuatorVelocity ||
                     midpoints != claim.joints)))
    return std::nullopt;
  claim.calibration = midpoints;
  return claim;
}

struct CommandModeSwitch {
  CommandClaim claim;
  bool reseed;
};

inline std::optional<CommandModeSwitch>
plan_command_mode_switch(CommandClaim current,
                         const std::vector<std::string> &start,
                         const std::vector<std::string> &stop) {
  const auto incoming = command_claim(start);
  const auto outgoing = command_claim(stop);
  if (!incoming || !outgoing)
    return std::nullopt;
  const bool starts = incoming->joints != 0;
  const bool stops = outgoing->joints != 0;
  if ((stops && !(*outgoing == current)) ||
      (starts && current.joints && !stops))
    return std::nullopt;
  return CommandModeSwitch{starts || stops ? *incoming : current,
                           starts || stops};
}

inline bool angles_ready(CommandClaim claim, unsigned valid_angles) {
  return !angular(claim.mode) || (claim.joints & valid_angles) == claim.joints;
}

} // namespace riser_kinematics
