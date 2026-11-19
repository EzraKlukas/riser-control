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
  double mass = 0.;      // kg. No UI for this - measured quantity.
  double height = 1.80;
  double is_male = true; // for gender-based constants.
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
    if (p.height <= 0 || p.mass <= 0)
      return "Height and mass must be positive.";

    Configuration next;
    next.viscosity = p.viscosity;
    next.mass = p.mass;
    next.com_height = (p.is_male ? 0.56 : 0.54) * p.height;
    next.inertia = (p.is_male ? 0.35 : 0.33) * p.mass * (p.height * p.height);
    configuration_ = next;
    configured_ = true;
    return nullptr;
  }

  const char *activate() {
    if (!configured_)
      return "Logic has not been configured";
    // This read-only provider starts invalid. Sensor readiness and timestamp
    // checks belong in update(), which receives the actual control-loop time.
    active_ = true;
    return nullptr;
  }

  const Configuration &configuration() const { return configuration_; }

  void deactivate() { active_ = false; }

  UpdateResult update(const Inputs &inputs, double time_s,
                      double period_s) const {
    UpdateResult result;
    // convention: dot, ddot = first, second derivative.
    double theta_bb = inputs.theta_bb.position;
    double theta_dot_bb = inputs.theta_bb.velocity;
    double theta_ddot_bb = 0.;

    const Configuration &c = configuration_;
    theta_ddot_bb = ((0.971 * c.mass * c.gravity * c.com_height) * theta_bb -
                     c.viscosity * theta_dot_bb + inputs.ankle_wrench.T_x) /
                    c.inertia;

    theta_dot_bb += theta_ddot_bb * dt;
    result.velocity_commands[THETA_BB] = theta_dot_bb;
    return result;
  }

private:
  bool active_ = false;
  bool configured_ = false;
  Configuration configuration_;
};
} // namespace riser_inverted_pendulum_controller::logic
