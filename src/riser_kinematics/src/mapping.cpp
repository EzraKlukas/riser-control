#include "riser_kinematics/mapping.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace riser_kinematics {
namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double domain_tolerance = 1e-12;
constexpr double closure_tolerance = 1e-10; // metres
constexpr double minimum_radicand = 1e-12;  // m^2, same as Python reference
constexpr double minimum_jacobian = 1e-8;   // same threshold as Python inverse
bool finite(const Vector3 &v) noexcept {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
double dot(const Vector3 &a, const Vector3 &b) noexcept {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
bool inside(double q, Domain d) noexcept {
  return q >= d.lower - domain_tolerance && q <= d.upper + domain_tolerance;
}
} // namespace

Sample Axis::raw(double q) const noexcept {
  const auto &g = geometry_;
  if (g.linear) {
    const double position = g.ratio * q;
    if (!std::isfinite(position))
      return {Error::NonfiniteInput};
    return {Error::None, position, 0.0, g.ratio};
  }
  const double angle = q + g.offset;
  const double sn = std::sin(angle), cs = std::cos(angle);
  Vector3 b{}, v{}, derivative{};
  for (std::size_t i = 0; i < 3; ++i)
    b[i] = g.body_pin[i] - g.pivot[i];
  v = {g.pivot[0] + b[0] - g.slider_reference[0],
       g.pivot[1] + cs * b[1] - sn * b[2] - g.slider_reference[1],
       g.pivot[2] + sn * b[1] + cs * b[2] - g.slider_reference[2]};
  derivative = {0.0, -sn * b[1] - cs * b[2], cs * b[1] - sn * b[2]};
  const double p = dot(g.slider_direction, v);
  const double dp = dot(g.slider_direction, derivative);
  const double D = g.rod_length * g.rod_length - dot(v, v) + p * p;
  if (!std::isfinite(D))
    return {Error::NonfiniteInput};
  if (D < 0.0)
    return {Error::Unreachable};
  if (D <= minimum_radicand)
    return {Error::Singular};
  const double root = std::sqrt(D);
  const double r = p + g.branch * root;
  const double J = dp + g.branch * (p * dp - dot(v, derivative)) / root;
  if (!std::isfinite(r) || !std::isfinite(J))
    return {Error::NonfiniteInput};
  return {Error::None, r, 0.0, J};
}

Error Axis::configure(const Geometry &g, Domain domain) noexcept {
  configured_ = false;
  if (!std::isfinite(domain.lower) || !std::isfinite(domain.upper) ||
      domain.lower >= domain.upper ||
      (!g.linear && domain.upper - domain.lower > 2 * pi))
    return Error::InvalidConfiguration;
  if (g.linear) {
    if (!std::isfinite(g.ratio) || std::abs(g.ratio) < minimum_jacobian)
      return Error::InvalidConfiguration;
  } else if (!finite(g.pivot) || !finite(g.body_pin) ||
             !finite(g.slider_reference) || !finite(g.slider_direction) ||
             !std::isfinite(g.offset) || !std::isfinite(g.rod_length) ||
             g.rod_length <= 0.0 || (g.branch != -1 && g.branch != 1) ||
             std::abs(dot(g.slider_direction, g.slider_direction) - 1.0) >
                 1e-9) {
    return Error::InvalidConfiguration;
  }
  geometry_ = g;
  domain_ = domain;
  const auto zero = raw(0.0);
  if (!zero)
    return zero.error;
  zero_ = zero.position;
  // Endpoint validation is not a claim that the entire domain is nonsingular.
  for (double q : {domain.lower, domain.upper}) {
    const auto endpoint = raw(q);
    if (!endpoint)
      return endpoint.error;
  }
  configured_ = true;
  return Error::None;
}

Sample Axis::forward(double q, double qdot) const noexcept {
  if (!configured_)
    return {Error::NotConfigured};
  if (!std::isfinite(q) || !std::isfinite(qdot))
    return {Error::NonfiniteInput};
  if (!inside(q, domain_))
    return {Error::OutsideDomain};
  auto result = raw(std::clamp(q, domain_.lower, domain_.upper));
  if (!result)
    return result;
  if (std::abs(result.jacobian) < minimum_jacobian)
    return {Error::Singular};
  result.position -= zero_;
  result.velocity = result.jacobian * qdot;
  if (!std::isfinite(result.position) || !std::isfinite(result.velocity))
    return {Error::NonfiniteInput};
  return result;
}

Sample Axis::inverse(double s, double seed, double sdot) const noexcept {
  if (!configured_)
    return {Error::NotConfigured};
  if (!std::isfinite(s) || !std::isfinite(seed) || !std::isfinite(sdot))
    return {Error::NonfiniteInput};
  if (!inside(seed, domain_))
    return {Error::OutsideDomain};
  if (geometry_.linear) {
    const double q = s / geometry_.ratio;
    const auto check = forward(q);
    if (!check)
      return {check.error};
    const double qdot = sdot / check.jacobian;
    if (!std::isfinite(qdot))
      return {Error::NonfiniteInput};
    return {Error::None, std::clamp(q, domain_.lower, domain_.upper), qdot,
            check.jacobian};
  }
  const auto &g = geometry_;
  Vector3 b{}, a{};
  for (std::size_t i = 0; i < 3; ++i) {
    b[i] = g.body_pin[i] - g.pivot[i];
    a[i] = g.pivot[i] - g.slider_reference[i] -
           (s + zero_) * g.slider_direction[i];
  }
  // Expand |a + Rx(q+offset)b|^2 = l^2. The x component does not rotate.
  const double A = 2 * (a[1] * b[1] + a[2] * b[2]);
  const double B = 2 * (-a[1] * b[2] + a[2] * b[1]);
  const double C =
      g.rod_length * g.rod_length - dot(a, a) - dot(b, b) - 2 * a[0] * b[0];
  const double radius = std::hypot(A, B);
  if (!std::isfinite(radius) || !std::isfinite(C))
    return {Error::NonfiniteInput};
  if (radius <= 1e-15)
    return {std::abs(C) <= 1e-15 ? Error::Singular : Error::Unreachable};
  const double cosine = C / radius;
  if (std::abs(cosine) > 1.0 + 1e-12)
    return {Error::Unreachable};
  const double phase = std::atan2(B, A);
  const double delta = std::acos(std::clamp(cosine, -1.0, 1.0));
  Sample best{Error::OutsideDomain};
  double distance = std::numeric_limits<double>::infinity();
  // A domain spans at most one revolution. Three neighboring lifts of each
  // analytic root suffice, including both endpoints; no iterative root search.
  for (double root : {phase - delta - g.offset, phase + delta - g.offset}) {
    const double middle = domain_.lower / 2 + domain_.upper / 2;
    const double turns = std::round((middle - root) / (2 * pi));
    for (int lift = -1; lift <= 1; ++lift) {
      const double candidate = root + (turns + lift) * 2 * pi;
      if (!inside(candidate, domain_))
        continue;
      const double q = std::clamp(candidate, domain_.lower, domain_.upper);
      const auto check = forward(q);
      if (!check) {
        if (distance == std::numeric_limits<double>::infinity())
          best.error = check.error;
        continue;
      }
      // Squaring the rod equation loses the assembly branch: verify against
      // the configured forward root before accepting an inverse candidate.
      if (std::abs(check.position - s) > closure_tolerance) {
        if (distance == std::numeric_limits<double>::infinity())
          best.error = Error::WrongBranch;
        continue;
      }
      if (std::abs(q - seed) < distance) {
        distance = std::abs(q - seed);
        best = {Error::None, q, sdot / check.jacobian, check.jacobian};
      }
    }
  }
  if (best && !std::isfinite(best.velocity))
    return {Error::NonfiniteInput};
  return best;
}

Error Mapping::configure(const std::array<Geometry, 4> &geometry,
                         const std::array<Domain, 4> &domains) noexcept {
  configured_ = false;
  for (std::size_t i = 0; i < 4; ++i) {
    const auto error = axes_[i].configure(geometry[i], domains[i]);
    if (error != Error::None)
      return error;
  }
  configured_ = true;
  return Error::None;
}
Error Mapping::forward(const Coordinates &q, const Coordinates &qdot,
                       Coordinates &s, Coordinates &sdot) const noexcept {
  if (!configured_)
    return Error::NotConfigured;
  Coordinates position{}, velocity{};
  for (std::size_t i = 0; i < 4; ++i) {
    const auto value = axes_[i].forward(q[i], qdot[i]);
    if (!value)
      return value.error;
    position[i] = value.position;
    velocity[i] = value.velocity;
  }
  s = position;
  sdot = velocity;
  return Error::None;
}
Error Mapping::inverse(const Coordinates &s, const Coordinates &sdot,
                       const Coordinates &seed, Coordinates &q,
                       Coordinates &qdot) const noexcept {
  if (!configured_)
    return Error::NotConfigured;
  Coordinates position{}, velocity{};
  for (std::size_t i = 0; i < 4; ++i) {
    const auto value = axes_[i].inverse(s[i], seed[i], sdot[i]);
    if (!value)
      return value.error;
    position[i] = value.position;
    velocity[i] = value.velocity;
  }
  q = position;
  qdot = velocity;
  return Error::None;
}
} // namespace riser_kinematics
