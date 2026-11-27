#include "../src/footplate_pitch.hpp"
#include <cmath>

int main() {
  using namespace riser_kinematics;
  constexpr Domain raw_limits{-0.45, 0.20};
  constexpr double midpoint = 0.05;
  Axis cad;
  if (cad.configure(canonical_geometry()[FootPitch], raw_limits) != Error::None)
    return 1;

  for (const double polarity : {1., -1.}) {
    riser_hardware::FootplatePitch pitch;
    if (!pitch.configure(raw_limits, 0.2, polarity, 2.))
      return 2;
    pitch.establish_reference(midpoint, 0.06);
    for (const double sign : {1., -1.}) {
      const auto raw = cad.forward(-sign * 0.01, -sign * 0.08);
      if (!raw)
        return 3;
      pitch.read(true, midpoint + polarity * raw.position,
                 polarity * raw.velocity);
      if (pitch.valid != 1 || std::abs(pitch.position - sign * 0.01) > 1e-9 ||
          std::abs(pitch.velocity - sign * 0.08) > 1e-9)
        return 4;

      pitch.velocity_command = sign * 0.08;
      pitch.position_command = sign * 0.01;
      const double velocity = pitch.carriage_command(false, midpoint, 1.);
      const double position = pitch.carriage_command(true, midpoint, 1.);
      if (velocity * polarity * sign <= 0 || position * polarity * sign <= 0 ||
          std::abs(velocity - polarity * raw.velocity) > 1e-9 ||
          std::abs(position - 2 * polarity * raw.position) > 1e-9)
        return 5;
    }
    pitch.read(true, midpoint, 0.);
    pitch.velocity_command = 1.;
    const double angular_limited = std::abs(pitch.jacobian) * 0.2;
    if (std::abs(std::abs(pitch.carriage_command(false, midpoint, 1.)) -
                 angular_limited) > 1e-9)
      return 8;
    const double carriage_cap = angular_limited / 2;
    if (std::abs(pitch.carriage_command(false, midpoint, carriage_cap)) != carriage_cap)
      return 9;

    pitch.read(true, midpoint, 0., 0., 1.);
    if (pitch.forward_limit != (polarity > 0) ||
        pitch.reverse_limit != (polarity < 0))
      return 6;
    pitch.read(true, midpoint, 0., 1., 0.);
    if (pitch.forward_limit != (polarity < 0) ||
        pitch.reverse_limit != (polarity > 0))
      return 7;
  }
}
