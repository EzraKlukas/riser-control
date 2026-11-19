#pragma once

#include <algorithm>

namespace riser_human_balance {

struct Pitch {
  double angle_rad;
  double rate_rad_s;
};

inline double ankle_torque_nm(Pitch body, double elapsed_s, double pulse_nm) {
  // Upright PD. The proportional gain exceeds the body's linearized
  // gravitational slope (roughly 580 N m/rad).
  constexpr double stiffness_nm_rad = 1250.;
  constexpr double damping_nm_s_rad = 310.;
  constexpr double limit_nm = 120.;
  constexpr double pulse_duration_s = 0.65;
  const double pulse = elapsed_s < pulse_duration_s ? pulse_nm : 0.;
  return std::clamp(pulse - stiffness_nm_rad * body.angle_rad -
                        damping_nm_s_rad * body.rate_rad_s,
                    -limit_nm, limit_nm);
}

inline double strap_torque_nm(Pitch body, Pitch board) {
  // Pitch-only coupling carries a moment between body and backboard without
  // supporting the body's weight. The ankle hinge supplies the reaction force.
  constexpr double stiffness_nm_rad = 4500.;
  constexpr double damping_nm_s_rad = 350.;
  constexpr double limit_nm = 160.;
  return std::clamp(stiffness_nm_rad * (board.angle_rad - body.angle_rad) +
                        damping_nm_s_rad * (board.rate_rad_s - body.rate_rad_s),
                    -limit_nm, limit_nm);
}

} // namespace riser_human_balance
