#pragma once

#include <algorithm>
#include <cmath>
#include <riser_kinematics/mapping.hpp>

namespace riser_hardware {
struct FootplatePitch {
  riser_kinematics::Axis mapping;
  double zero_m = 0, direction = 0, lower_m = 0, upper_m = 0;
  double rate_limit = 0, position_gain = 0;
  double position = 0, velocity = 0, valid = 0, jacobian = 0;
  double position_command = 0, velocity_command = 0;

  bool configure(riser_kinematics::Domain angles, double rate,
                 double zero, double lower, double upper, double sign,
                 double gain) {
    using namespace riser_kinematics;
    if (!(angles.lower < 0 && angles.upper > 0) ||
        !(lower < zero && zero < upper) || (sign != 1 && sign != -1) ||
        !(rate > 0) || gain < 0 ||
        mapping.configure(canonical_geometry()[FootPitch], angles) != Error::None)
      return false;
    const auto lo = mapping.forward(angles.lower);
    const auto hi = mapping.forward(angles.upper);
    if (!lo || !hi || lo.jacobian * hi.jacobian <= 0)
      return false;
    const auto [cad_min, cad_max] =
        std::minmax({zero + sign * lo.position, zero + sign * hi.position});
    lower_m = std::max(lower, cad_min);
    upper_m = std::min(upper, cad_max);
    zero_m = zero;
    direction = sign;
    rate_limit = rate;
    position_gain = gain;
    return true;
  }

  void read(bool feedback, double carriage_m, double carriage_rate) {
    if (direction && feedback) {
      const auto angle = mapping.inverse(direction * (carriage_m - zero_m),
                                         position, direction * carriage_rate);
      if (angle) {
        position = angle.position;
        velocity = angle.velocity;
        jacobian = direction * angle.jacobian; // Encoder metres per radian.
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
      const auto target = mapping.forward(position_command);
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
