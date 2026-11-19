#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace riser_inverted_pendulum_controller {

inline constexpr std::size_t kNumAngularJoints = 3;
enum JointIndices { THETA_BB, THETA_ML, THETA_FP };

} // namespace riser_inverted_pendulum_controller

namespace riser_inverted_pendulum_controller::logic {

inline constexpr double Unavailable = std::numeric_limits<double>::quiet_NaN();
inline constexpr double dt = 0.001; // seconds

struct TorqueSample {
  double T_x, T_y, sample_time_s = Unavailable;
  bool valid = false;
};

struct JointSample {
  double position, velocity, effort = Unavailable;
};

struct Inputs {
  TorqueSample ankle_wrench;
  JointSample theta_ml, theta_fp, theta_bb; // bb backboard, fp footplate
};

struct UpdateResult {
  std::array<double, kNumAngularJoints> velocity_commands{};
  const char *error = nullptr;
};

struct Parameters {
  double viscosity = 0.; // kg / (m s)
  double mass = 0.;      // kg. Zero selects the provider's measured mass.
  double height = 1.80;
  bool is_male = true;   // for gender-based constants.
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
  double com_height = 0.; // relative to floor

  bool ml = true;
  bool ap = true;
};

class InvertedPendulumLogic {
public:
  const char *configure(const Parameters &p) {
    if (active_)
      return "Deactivate before configuring";
    if (!std::isfinite(p.height) || !std::isfinite(p.viscosity) ||
        !std::isfinite(p.mass))
      return "Height, viscosity, mass must be finite.";
    if (p.height <= 0 || p.mass < 0)
      return "Height must be positive and the mass override nonnegative.";

    parameters_ = p;
    configured_ = true;
    return nullptr;
  }

  const char *activate(double measured_mass_kg) {
    if (!configured_)
      return "Logic has not been configured";
    const auto &p = parameters_;
    const double mass = p.mass > 0. ? p.mass : measured_mass_kg;
    if (!std::isfinite(mass) || mass <= 0.)
      return "Measure participant mass with the ankle wrench controller before activating";
    Configuration next;
    next.viscosity = p.viscosity;
    next.mass = mass;
    next.com_height = (p.is_male ? 0.56 : 0.54) * p.height;
    next.inertia = (p.is_male ? 0.35 : 0.33) * mass * (p.height * p.height);
    if (!std::isfinite(next.inertia) || next.inertia <= 0.)
      return "Calculated inertia must be positive and finite";
    configuration_ = next;
    active_ = true;
    return nullptr;
  }

  const Configuration &configuration() const { return configuration_; }

  void deactivate() { active_ = false; }

  UpdateResult update(const Inputs &inputs, double time_s,
                      double period_s) const {
    UpdateResult result;
    if (!active_ || !inputs.ankle_wrench.valid ||
        !std::isfinite(inputs.ankle_wrench.T_x) ||
        !std::isfinite(inputs.ankle_wrench.sample_time_s) ||
        !std::isfinite(inputs.theta_bb.position) ||
        !std::isfinite(inputs.theta_bb.velocity) ||
        !std::isfinite(time_s) || !std::isfinite(period_s) || period_s < 0.) {
      result.error = "Invalid state for the inverted pendulum";
      return result;
    }
    // convention: dot, ddot = first, second derivative.
    double theta_bb = inputs.theta_bb.position;
    double theta_dot_bb = inputs.theta_bb.velocity;
    double theta_ddot_bb = 0.;

    const Configuration &c = configuration_;
    theta_ddot_bb = ((0.971 * c.mass * c.gravity * c.com_height) * theta_bb -
                     c.viscosity * theta_dot_bb + inputs.ankle_wrench.T_x) /
                    c.inertia;

    theta_dot_bb += theta_ddot_bb * dt;
    if (!std::isfinite(theta_dot_bb)) {
      result.error = "Calculated backboard command is nonfinite";
      return result;
    }
    result.velocity_commands[THETA_BB] = theta_dot_bb;
    return result;
  }

private:
  bool active_ = false;
  bool configured_ = false;
  Parameters parameters_;
  Configuration configuration_;
};
} // namespace riser_inverted_pendulum_controller::logic
