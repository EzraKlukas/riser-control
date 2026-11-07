#pragma once

#include <array>
#include <cstddef>

namespace riser_kinematics {
// Independent coordinates, NOT [backboard_pitch, foot_plate_pitch, ml_angle].
enum AxisIndex : std::size_t {
  FramePitch,
  TorsoLateral,
  HipLateral,
  FootPitch
};
using Coordinates = std::array<double, 4>;
using Vector3 = std::array<double, 3>;

struct Domain {
  double lower, upper;
}; // radians for pitch; metres for slides
struct Geometry {
  bool linear = false;
  double ratio = 1.0;
  double offset = 0.0;
  Vector3 pivot{}, body_pin{}, slider_reference{}, slider_direction{};
  double rod_length = 0.0;
  int branch = 1;
};

enum class Error {
  None,
  NotConfigured,
  InvalidConfiguration,
  NonfiniteInput,
  OutsideDomain,
  Unreachable,
  Singular,
  WrongBranch
};
struct Sample {
  Error error = Error::None;
  double position = 0.0;
  double velocity = 0.0;
  double jacobian = 0.0; // ds/dq, including for inverse results
  explicit operator bool() const noexcept { return error == Error::None; }
};

// No allocation, exceptions, IO or mutable history in evaluation calls.
// configure() invalidates the old configuration on failure. Configure only
// while evaluation is stopped; seed continuity is explicitly owned by the
// caller.
class Axis {
public:
  Error configure(const Geometry &geometry, Domain domain) noexcept;
  Sample forward(double q, double qdot = 0.0) const noexcept;
  Sample inverse(double s, double seed, double sdot = 0.0) const noexcept;

private:
  Sample raw(double q) const noexcept;
  Geometry geometry_{};
  Domain domain_{};
  double zero_ = 0.0;
  bool configured_ = false;
};

// Geometry is generated from evidence/model_reference/actuator_mapping.json.
std::array<Geometry, 4> canonical_geometry() noexcept;

class Mapping {
public:
  Error configure(const std::array<Geometry, 4> &geometry,
                  const std::array<Domain, 4> &domains) noexcept;
  Error forward(const Coordinates &q, const Coordinates &qdot, Coordinates &s,
                Coordinates &sdot) const noexcept;
  Error inverse(const Coordinates &s, const Coordinates &sdot,
                const Coordinates &seed, Coordinates &q,
                Coordinates &qdot) const noexcept;

private:
  std::array<Axis, 4> axes_{};
  bool configured_ = false;
};
} // namespace riser_kinematics
