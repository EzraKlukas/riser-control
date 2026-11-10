#pragma once

#include <algorithm>
#include <array> #include <cmath>
#include <riser_kinematics/lateral.hpp>

namespace riser_gazebo {
namespace kin = riser_kinematics;

// Exported resources: angles [rad] first, then neutral-relative actuator
// strokes [m].
enum PublicJoint : std::size_t {
  Backboard,
  FootPlate,
  Ml,
  TopActuator,
  TorsoActuator,
  HipActuator,
  BottomActuator,
  PublicJointCount
};
// Actual Gazebo joints; the pitch actuator stages are not simulated bodies.
enum PhysicalJoint : std::size_t {
  BackboardJoint,
  FootPlateJoint,
  TorsoJoint,
  HipJoint,
  PhysicalJointCount
};
constexpr std::size_t AngularCount = 3, ActuatorCount = 4;
using PublicCoordinates = std::array<double, PublicJointCount>;
constexpr std::array<std::size_t, 4> PublicForAxis{TopActuator, TorsoActuator,
                                                   HipActuator, BottomActuator};
constexpr std::array<std::size_t, 4> PhysicalForAxis{BackboardJoint, TorsoJoint,
                                                     HipJoint, FootPlateJoint};
// All four-element mapping/reference arrays use kin::AxisIndex, NOT Gazebo
// order.
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
struct JointLimit {
  double lower, upper, velocity;
};
using Limits = std::array<JointLimit, PublicJointCount>;

// ROS-independent reference calculation, shared by the plugin and its tests.
// Configuration is done before activation. Evaluation is bounded/noexcept and
// commits outputs only after all mapping and limit checks succeed.
class ReferenceMapping {
public:
  kin::Error configure(const Limits &limits, std::array<double, 2> heights,
                       const kin::Coordinates &neutral_cad) noexcept {
    ready_ = false;
    for (const auto &l : limits)
      if (!std::isfinite(l.lower) || !std::isfinite(l.upper) ||
          !std::isfinite(l.velocity) || l.lower >= l.upper || l.velocity <= 0)
        return kin::Error::InvalidConfiguration;
    limits_ = limits;
    neutral_cad_ = neutral_cad;
    const auto geometry = kin::canonical_geometry();
    const auto slide_domain = [&](std::size_t axis) {
      const auto &limit = limits[PublicForAxis[axis]];
      const double a = limit.lower / geometry[axis].ratio;
      const double b = limit.upper / geometry[axis].ratio;
      return kin::Domain{std::min(a, b), std::max(a, b)};
    };
    const std::array<kin::Domain, 4> domains{
        {{limits[Backboard].lower, limits[Backboard].upper},
         slide_domain(kin::TorsoLateral),
         slide_domain(kin::HipLateral),
         {limits[FootPlate].lower, limits[FootPlate].upper}}};
    for (std::size_t i = 0; i < ActuatorCount; ++i) {
      // Geometry stays in its audited CAD frame. Only this adapter changes
      // zeros: q_CAD=q+neutral; s=f(q_CAD)-f(neutral). No encoder offsets here.
      if (!std::isfinite(neutral_cad[i]))
        return kin::Error::InvalidConfiguration;
      const auto domain = domains[i];
      physical_domains_[i] = domain;
      const auto error =
          axes_[i].configure(geometry[i], {domain.lower + neutral_cad[i],
                                           domain.upper + neutral_cad[i]});
      if (error != kin::Error::None)
        return error;
      const auto zero = axes_[i].forward(neutral_cad[i]);
      if (!zero)
        return zero.error;
      neutral_stroke_[i] = zero.position;
    }
    const auto error =
        ml_.configure(heights, {limits[Ml].lower, limits[Ml].upper});
    ready_ = error == kin::Error::None;
    return error;
  }

  // q is [backboard angle, torso slide, hip slide, footplate angle], from
  // operational neutral. Return s=f(q+neutral)-f(neutral), sdot=J*qdot and
  // ds/dq. In particular, the canonical torso/hip stroke signs are opposite to
  // slides.
  kin::Error observe(const kin::Coordinates &q, const kin::Coordinates &qdot,
                     PublicCoordinates &position, PublicCoordinates &rate,
                     kin::Coordinates &jacobian) const noexcept {
    if (!ready_)
      return kin::Error::NotConfigured;
    PublicCoordinates p{}, v{};
    kin::Coordinates J{};
    p[Backboard] = q[kin::FramePitch];
    p[FootPlate] = q[kin::FootPitch];
    v[Backboard] = qdot[kin::FramePitch];
    v[FootPlate] = qdot[kin::FootPitch];
    for (std::size_t i = 0; i < ActuatorCount; ++i) {
      if (!std::isfinite(q[i]))
        return kin::Error::NonfiniteInput;
      // Contact solvers can overshoot a physical stop. For measurement only,
      // extend the endpoint tangent so switches remain observable there.
      // Reference conversion below retains its strict mechanical bounds.
      const double bounded_q = std::clamp(q[i], physical_domains_[i].lower,
                                          physical_domains_[i].upper);
      const auto mapped =
          axes_[i].forward(bounded_q + neutral_cad_[i], qdot[i]);
      if (!mapped)
        return mapped.error;
      p[PublicForAxis[i]] = mapped.position - neutral_stroke_[i] +
                            mapped.jacobian * (q[i] - bounded_q);
      if (!std::isfinite(p[PublicForAxis[i]]))
        return kin::Error::NonfiniteInput;
      v[PublicForAxis[i]] = mapped.velocity;
      J[i] = mapped.jacobian;
    }
    kin::MlState fitted;
    const auto error =
        ml_.inverse({{q[kin::TorsoLateral], q[kin::HipLateral]},
                     {qdot[kin::TorsoLateral], qdot[kin::HipLateral]}},
                    fitted);
    if (error != kin::Error::None)
      return error;
    p[Ml] = fitted.position;
    v[Ml] = fitted.velocity;
    position = p;
    rate = v;
    jacobian = J;
    return kin::Error::None;
  }

  kin::Error update(CommandMode mode, const PublicCoordinates &command,
                    double dt, PublicCoordinates &position,
                    PublicCoordinates &rate, kin::Coordinates &physical,
                    kin::Coordinates &physical_rate) const noexcept {
    if (!ready_)
      return kin::Error::NotConfigured;
    if (mode == CommandMode::None)
      return kin::Error::InvalidConfiguration;
    if (!std::isfinite(dt) || dt <= 0)
      return kin::Error::NonfiniteInput;
    auto p = position, v = rate;
    auto q = physical, qdot = physical_rate;

    // 0. Validate/integrate ONLY the commanded coordinate family. A velocity
    // command is integrated once in its native units. Position commands have
    // zero reference velocity, matching the existing position-servo behavior.
    const std::size_t first = angular(mode) ? Backboard : TopActuator;
    const std::size_t last = angular(mode) ? TopActuator : PublicJointCount;
    for (std::size_t i = first; i < last; ++i) {
      if (!std::isfinite(command[i]))
        return kin::Error::NonfiniteInput;
      const auto &limit = limits_[i];
      if (velocity(mode)) {
        if (!std::isfinite(p[i]))
          return kin::Error::NonfiniteInput;
        v[i] = std::clamp(command[i], -limit.velocity, limit.velocity);
        p[i] = std::clamp(p[i] + dt * v[i], limit.lower, limit.upper);
        if (p[i] == limit.lower || p[i] == limit.upper)
          v[i] = 0;
      } else {
        p[i] = command[i];
        v[i] = 0;
        if (!bounded(i, p[i], v[i]))
          return kin::Error::OutsideDomain;
      }
    }

    if (!angular(mode)) {
      // 0.5. inverse(s, seed_q, sdot) returns q=f^-1(s), qdot=sdot/J(q).
      // Previous physical references seed the branch-continuous inverse. This
      // converts BOTH pitch targets and rates; do not integrate them again.
      // Slides remain independent q values -- fitting ML must not recouple
      // them.
      for (std::size_t i = 0; i < ActuatorCount; ++i) {
        const auto j = PublicForAxis[i];
        const auto mapped = axes_[i].inverse(
            p[j] + neutral_stroke_[i], physical[i] + neutral_cad_[i], v[j]);
        if (!mapped)
          return mapped.error;
        q[i] = mapped.position - neutral_cad_[i];
        qdot[i] = mapped.velocity;
      }
      p[Backboard] = q[kin::FramePitch];
      v[Backboard] = qdot[kin::FramePitch];
      p[FootPlate] = q[kin::FootPitch];
      v[FootPlate] = qdot[kin::FootPitch];
      kin::MlState fitted;
      const auto error =
          ml_.inverse({{q[kin::TorsoLateral], q[kin::HipLateral]},
                       {qdot[kin::TorsoLateral], qdot[kin::HipLateral]}},
                      fitted);
      if (error != kin::Error::None)
        return error;
      p[Ml] = fitted.position;
      v[Ml] = fitted.velocity; // diagnostic only here
    } else {
      // 1. Angular references were already checked/integrated above. Expand ML
      // to physical slides; pitch coordinates remain independent of each other.
      kin::SlideState slides;
      const auto error = ml_.forward(p[Ml], v[Ml], slides);
      if (error != kin::Error::None)
        return error;
      q = {p[Backboard], slides.position[0], slides.position[1], p[FootPlate]};
      qdot = {v[Backboard], slides.velocity[0], slides.velocity[1],
              v[FootPlate]};
      for (std::size_t i = 0; i < ActuatorCount; ++i) {
        const auto mapped = axes_[i].forward(q[i] + neutral_cad_[i], qdot[i]);
        if (!mapped)
          return mapped.error;
        p[PublicForAxis[i]] = mapped.position - neutral_stroke_[i];
        v[PublicForAxis[i]] = mapped.velocity;
      }
    }
    // Recheck transformed references in both unit systems. Reject an invalid
    // conversion rather than clamping qdot independently of its actuator rate.
    if (!bounded(Backboard, p[Backboard], v[Backboard]) ||
        !bounded(FootPlate, p[FootPlate], v[FootPlate]))
      return kin::Error::OutsideDomain;
    for (const auto j : PublicForAxis)
      if (!bounded(j, p[j], v[j]))
        return kin::Error::OutsideDomain;
    position = p;
    rate = v;
    physical = q;
    physical_rate = qdot;
    return kin::Error::None;
  }

private:
  bool bounded(std::size_t i, double p, double v) const noexcept {
    constexpr double tolerance = 1e-10; // mapping roundoff, not extra travel
    const auto &l = limits_[i];
    return std::isfinite(p) && std::isfinite(v) && p >= l.lower - tolerance &&
           p <= l.upper + tolerance && std::abs(v) <= l.velocity + tolerance;
  }
  Limits limits_{};
  std::array<kin::Axis, 4> axes_{};
  std::array<kin::Domain, 4> physical_domains_{};
  kin::Coordinates neutral_cad_{}, neutral_stroke_{};
  kin::LateralCoupling ml_;
  bool ready_ = false;
};
} // namespace riser_gazebo
