#include "riser_kinematics/lateral.hpp"
#include <cmath>

namespace riser_kinematics {
Error LateralCoupling::configure(std::array<double, 2> heights,
                                 Domain domain) noexcept {
  configured_ = false;
  constexpr double half_pi = 1.57079632679489661923;
  if (!std::isfinite(heights[0]) || !std::isfinite(heights[1]) ||
      heights[0] <= 0 || heights[1] <= 0 || !std::isfinite(domain.lower) ||
      !std::isfinite(domain.upper) || domain.lower >= domain.upper ||
      domain.lower <= -half_pi || domain.upper >= half_pi)
    return Error::InvalidConfiguration;
  const double squared = heights[0] * heights[0] + heights[1] * heights[1];
  if (!std::isfinite(squared) || squared == 0)
    return Error::InvalidConfiguration;
  heights_ = heights;
  domain_ = domain;
  squared_height_ = squared;
  configured_ = true;
  return Error::None;
}
Error LateralCoupling::forward(double angle, double rate,
                               SlideState &slides) const noexcept {
  if (!configured_)
    return Error::NotConfigured;
  if (!std::isfinite(angle) || !std::isfinite(rate))
    return Error::NonfiniteInput;
  if (angle < domain_.lower || angle > domain_.upper)
    return Error::OutsideDomain;
  const double tangent = std::tan(angle);
  SlideState result;
  for (std::size_t i = 0; i < 2; ++i) {
    result.position[i] = heights_[i] * tangent;
    result.velocity[i] = heights_[i] * (1 + tangent * tangent) * rate;
    if (!std::isfinite(result.position[i]) ||
        !std::isfinite(result.velocity[i]))
      return Error::NonfiniteInput;
  }
  slides = result;
  return Error::None;
}
Error LateralCoupling::inverse(const SlideState &slides,
                               MlState &ml) const noexcept {
  if (!configured_)
    return Error::NotConfigured;
  for (std::size_t i = 0; i < 2; ++i)
    if (!std::isfinite(slides.position[i]) ||
        !std::isfinite(slides.velocity[i]))
      return Error::NonfiniteInput;
  const double ht = heights_[0], hh = heights_[1];
  MlState result;
  result.tangent =
      (ht * slides.position[0] + hh * slides.position[1]) / squared_height_;
  result.position = std::atan(result.tangent);
  result.velocity = (ht * slides.velocity[0] + hh * slides.velocity[1]) /
                    (squared_height_ * (1 + result.tangent * result.tangent));
  const double length = std::sqrt(squared_height_);
  result.position_residual =
      (hh * slides.position[0] - ht * slides.position[1]) / length;
  result.velocity_residual =
      (hh * slides.velocity[0] - ht * slides.velocity[1]) / length;
  if (!std::isfinite(result.tangent) || !std::isfinite(result.velocity) ||
      !std::isfinite(result.position_residual) ||
      !std::isfinite(result.velocity_residual))
    return Error::NonfiniteInput;
  ml = result;
  return Error::None;
}
} // namespace riser_kinematics
