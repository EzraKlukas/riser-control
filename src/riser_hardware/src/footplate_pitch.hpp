#pragma once

#include <algorithm>
#include <cmath>
#include <tuple>
#include <riser_kinematics/mapping.hpp>

namespace riser_hardware {
struct FootplatePitch {
  riser_kinematics::Axis mapping;
  double zero_m = 0, direction = 0, lower_m = 0, upper_m = 0;
  double cad_lower_m = 0, cad_upper_m = 0;
  double rate_limit = 0, position_gain = 0;
  double position = 0, velocity = 0, valid = 0, jacobian = 0;
  double forward_limit = 0, reverse_limit = 0;
  double position_command = 0, velocity_command = 0;
  bool homed = false;

  bool configure(riser_kinematics::Domain angles, double rate,
                 double sign, double gain) {
    using namespace riser_kinematics;
    if (!(angles.lower < 0 && angles.upper > 0) ||
        (sign != 1 && sign != -1) || !(rate > 0) || gain < 0 ||
        mapping.configure(canonical_geometry()[FootPitch], angles) != Error::None)
      return false;
    const auto lo = mapping.forward(angles.lower);
    const auto hi = mapping.forward(angles.upper);
    if (!lo || !hi || lo.jacobian * hi.jacobian <= 0)
      return false;
    std::tie(cad_lower_m, cad_upper_m) =
        std::minmax({sign * lo.position, sign * hi.position});
    direction = sign;
    rate_limit = rate;
    position_gain = gain;
    return true;
  }

  void establish_reference(double midpoint, double half_travel) {
    homed = std::isfinite(midpoint) && std::isfinite(half_travel) && half_travel > 0;
    if (!homed)
      return;
    zero_m = midpoint;
    lower_m = midpoint + std::max(-half_travel, cad_lower_m);
    upper_m = midpoint + std::min(half_travel, cad_upper_m);
  }

  void read(bool feedback, double carriage_m, double carriage_rate,
            double reverse = 0, double forward = 0) {
    homed &= feedback;
    const auto neutral = mapping.forward(0.);
    const double mechanism_sign = riser_kinematics::kFootPitchMechanismSign;
    const bool positive = neutral && direction * neutral.jacobian * mechanism_sign > 0;
    forward_limit = positive ? forward : reverse;
    reverse_limit = positive ? reverse : forward;
    if (homed) {
      const auto angle = mapping.inverse(direction * (carriage_m - zero_m),
                                         mechanism_sign * position, direction * carriage_rate);
      if (angle) {
        position = mechanism_sign * angle.position;
        velocity = mechanism_sign * angle.velocity;
        jacobian = direction * angle.jacobian * mechanism_sign; // Encoder metres per mechanism radian.
        valid = 1;
        return;
      }
    }
    position = velocity = valid = jacobian = 0;
  }

  double carriage_command(bool use_position, double carriage_m,
                          double max_speed) const {
    if (!valid)
      return NAN;
    double speed = jacobian * velocity_command;
    if (use_position) {
      const auto target = mapping.forward(
          riser_kinematics::kFootPitchMechanismSign * position_command);
      if (!target)
        return NAN;
      const double setpoint = std::clamp(zero_m + direction * target.position,
                                         lower_m, upper_m);
      speed = (setpoint - carriage_m) * position_gain;
    }
    if (!std::isfinite(speed))
      return NAN;
    if ((speed < 0 && carriage_m <= lower_m) ||
        (speed > 0 && carriage_m >= upper_m))
      return 0;
    const double limit = std::min(max_speed, std::abs(jacobian) * rate_limit);
    return std::clamp(speed, -limit, limit);
  }
};
} // namespace riser_hardware
