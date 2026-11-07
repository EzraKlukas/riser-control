#pragma once
#include "riser_kinematics/mapping.hpp"

namespace riser_kinematics {
// Physical slide coordinates (metres), BEFORE the -1 actuator-stroke ratios.
struct SlideState {
  std::array<double, 2> position{}, velocity{}; // torso, hip
};
struct MlState {
  double position = 0.0, velocity = 0.0, tangent = 0.0;
  // Signed orthogonal residuals in m and m/s. Nonzero means the independent
  // slides do not represent one common ML motion; no calibration is inferred.
  double position_residual = 0.0, velocity_residual = 0.0;
};
class LateralCoupling {
public:
  Error configure(std::array<double, 2> heights, Domain angle_domain) noexcept;
  Error forward(double angle, double rate, SlideState &slides) const noexcept;
  Error inverse(const SlideState &slides, MlState &ml) const noexcept;

private:
  std::array<double, 2> heights_{};
  Domain domain_{};
  double squared_height_ = 0.0;
  bool configured_ = false;
};
} // namespace riser_kinematics
