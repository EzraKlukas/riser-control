#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace riser_inverted_pendulum_controller::logic {

inline constexpr double Unavailable = std::numeric_limits<double>::quiet_NaN();

struct TorqueSample {
  double T_x, T_y, sample_time_s = Unavailable;
  bool valid = false;
};

struct JointSample {
  double position, velocity, effort = Unavailable;
};

struct Inputs {
  TorqueSample ankle_wrench;
  JointSample angle;
  bool reverse_limit = false, forward_limit = false;
};

struct UpdateResult {
  double velocity_command = 0;
  const char *error = nullptr;
};

struct Parameters {
  double viscosity = 0.; // N m s / rad
  double height = 1.80;
  double max_rate_rad_s = 0.; // Set from the configured robot model.
  double lower_pendulum_angle_rad = -0.12, upper_pendulum_angle_rad = 0.12;
  bool is_male = true; // for gender-based constants.
  // to add and implement later: inertia gain, footplate scaling, backboard
  // scaling, ml scaling, b scaling, gravity, halt-based params, ankle width,
  // stance width, soft upper and lower limits, load stiffness gain, delay_AP,
  // delay_ML
};

struct Configuration {
  double viscosity = 0.;
  double gravity = 9.81;
  double mass = 0.;
  double inertia = 0.;
  double max_rate_rad_s = 0.;
  double com_height = 0.; // relative to floor

  bool ml = true;
  bool ap = true;
};

class InvertedPendulumLogic {
public:
  const char *configure(const Parameters &p) {
    if (active_)
      return "Deactivate before configuring";
    if (!std::isfinite(p.height) || !std::isfinite(p.viscosity))
      return "Height and viscosity must be finite.";
    if (p.height <= 0)
      return "Height must be positive.";
    if (!std::isfinite(p.max_rate_rad_s) || p.max_rate_rad_s <= 0.)
      return "Maximum angular rate must be positive and finite.";

    if (!std::isfinite(p.lower_pendulum_angle_rad) ||
        !std::isfinite(p.upper_pendulum_angle_rad) ||
        p.lower_pendulum_angle_rad >= 0 || p.upper_pendulum_angle_rad <= 0)
      return "Pendulum angle limits must be finite and straddle zero";
    parameters_ = p;
    configured_ = true;
    return nullptr;
  }

  const char *activate(double mass) {
    if (!configured_)
      return "Logic has not been configured";
    const auto &p = parameters_;
    if (!std::isfinite(mass) || mass <= 0.)
      return "Set a positive mass in kilograms or use "
             "/ankle_wrench/measure_mass before activating";
    Configuration next;
    next.viscosity = p.viscosity;
    next.mass = mass;
    next.max_rate_rad_s = p.max_rate_rad_s;
    next.com_height = (p.is_male ? 0.56 : 0.54) * p.height;
    next.inertia = (p.is_male ? 0.35 : 0.33) * mass * (p.height * p.height);
    if (!std::isfinite(next.inertia) || next.inertia <= 0.)
      return "Calculated inertia must be positive and finite";
    configuration_ = next;
    rate_command_rad_s_ = 0.;
    active_ = true;
    return nullptr;
  }

  const Configuration &configuration() const { return configuration_; }

  void deactivate() { active_ = false; }

  UpdateResult update(const Inputs &inputs, double time_s, double period_s) {
    UpdateResult result;
    if (!active_ || !inputs.ankle_wrench.valid ||
        !std::isfinite(inputs.ankle_wrench.T_x) ||
        !std::isfinite(inputs.ankle_wrench.sample_time_s) ||
        !std::isfinite(inputs.angle.position) ||
        !std::isfinite(inputs.angle.velocity) || !std::isfinite(time_s) ||
        !std::isfinite(period_s) || period_s < 0.) {
      result.error = "Invalid state for the inverted pendulum";
      return result;
    }
    /* NO ERROR FOR HITTING THE SOFTWARE LIMIT!
if (inputs.angle.position <= parameters_.lower_pendulum_angle_rad ||
    inputs.angle.position >= parameters_.upper_pendulum_angle_rad) {
  result.error = "Pendulum angle reached its software limit";
  return result;
}
    */
    const Configuration &c = configuration_;

    const double torque_on_participant = -1.0 * inputs.ankle_wrench.T_x;

    // get the actual angle in that we selected for soft limit in place of
    // reverse/forward_limit
    const double theta_b =
        std::clamp(inputs.angle.position, parameters_.lower_pendulum_angle_rad,
                   parameters_.upper_pendulum_angle_rad); // 0.02 rad
    const double alpha =
        std::clamp(std::abs(inputs.angle.position - theta_b) / 0.0174, 0.,
                   1.); // linearly increasing once we get above 0.02 rad

    const double s =
        2 * (inputs.angle.position > 0) - 1.0; // the sign, is positive, so +1

    const double torque_user =
        (s * torque_on_participant > 0)
            ? (1 - alpha) *
                  torque_on_participant // negative, much bigger than one
            : inputs.ankle_wrench.T_x;

    const double w_sign = 2 * (inputs.angle.velocity > 0) - 1;
    const double damping_contribution =
        w_sign * s > 0 ? 15 * s * c.inertia * inputs.angle.velocity : 0.;

    const double torque_out =
        torque_user - alpha * (0.971 * c.mass * c.gravity * c.com_height *
                                   inputs.angle.position +
                               damping_contribution);

    // The sensed reaction torque has the opposite sign to the model input.
    const double acceleration_rad_s2 =
        (0.971 * c.mass * c.gravity * c.com_height * inputs.angle.position -
         c.viscosity * rate_command_rad_s_ + torque_out) /
        c.inertia;

    const double next_rate_rad_s =
        rate_command_rad_s_ + acceleration_rad_s2 * period_s;
    if (!std::isfinite(next_rate_rad_s)) {
      result.error = "Calculated angular command is nonfinite";
      return result;
    }
    // Keep the applied model rate as state so a saturated command cannot
    // accumulate a larger rate behind the Gazebo actuator limit.
    rate_command_rad_s_ =
        std::clamp(next_rate_rad_s, -c.max_rate_rad_s, c.max_rate_rad_s);
    if ((rate_command_rad_s_ < 0 && inputs.reverse_limit) ||
        (rate_command_rad_s_ > 0 && inputs.forward_limit))
      rate_command_rad_s_ = 0;
    result.velocity_command = rate_command_rad_s_;
    return result;
  }

private:
  bool active_ = false;
  bool configured_ = false;
  double rate_command_rad_s_ = 0.;
  Parameters parameters_;
  Configuration configuration_;
};
} // namespace riser_inverted_pendulum_controller::logic
