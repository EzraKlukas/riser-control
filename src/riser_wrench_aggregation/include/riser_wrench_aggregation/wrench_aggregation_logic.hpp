#pragma once

#include "riser_wrench_aggregation/transformations.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace riser_wrench_aggregation::logic {
using wrench_transform::Matrix3;
using wrench_transform::Vector3;
using wrench_transform::Wrench;
inline constexpr double Unavailable = std::numeric_limits<double>::quiet_NaN();
// Nanosecond timestamps can round differently when converted to double seconds.
inline constexpr double TimestampToleranceS = 1e-9;

struct WrenchSample {
  Wrench wrench{{Unavailable, Unavailable, Unavailable},
                {Unavailable, Unavailable, Unavailable}};
  double sample_time_s = Unavailable;
  bool valid = false;
};

struct Inputs {
  WrenchSample left;
  WrenchSample right;
};

// Variable-length parameter values are checked before conversion to fixed
// sizes.
struct PlateParameters {
  std::vector<double> ankle_to_origin;
  std::vector<double> linear_transform{1., 0., 0., 0., 1., 0., 0., 0., 1.};
  double gain = 1.;
};

struct Parameters {
  PlateParameters left{{0.127, 0.1016, 0.08509}};
  PlateParameters right{{-0.127, 0.1016, 0.08509}};
  std::vector<double> symmetric_transform{1., 0., 0., 0., 1., 0., 0., 0., 1.};
  double ankle_gain = 1.;
  bool same_transformation = true;
  bool same_scaling = true;
  bool custom_transformation = true;
  bool matrix_transformation = true;
  bool scaling = true;
  double max_sample_age_s = 0.1;
  double max_sample_skew_s = 0.01;
};

struct PlateConfiguration {
  Vector3 ankle_to_origin{};
  Matrix3 linear_transform = wrench_transform::Identity;
  double gain = 1.;
};

struct Configuration {
  PlateConfiguration left, right;
  Matrix3 symmetric_transform = wrench_transform::Identity;
  double ankle_gain = 1.;
  bool same_transformation = true;
  bool same_scaling = true;
  bool custom_transformation = true;
  bool matrix_transformation = true;
  bool scaling = true;
  double max_sample_age_s = 0.1;
  double max_sample_skew_s = 0.01;
};

struct UpdateResult {
  WrenchSample output;
  const char *error = nullptr;
};

inline bool finite(const Wrench &wrench) {
  const auto valid = [](double value) { return std::isfinite(value); };
  return std::all_of(wrench.force.begin(), wrench.force.end(), valid) &&
         std::all_of(wrench.torque.begin(), wrench.torque.end(), valid);
}

inline bool finite_vector(const std::vector<double> &values, std::size_t size) {
  return values.size() == size &&
         std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

inline Matrix3 matrix_from_values(const std::vector<double> &values) {
  Matrix3 matrix{};
  for (std::size_t row = 0; row < matrix.size(); ++row)
    std::copy_n(values.begin() + 3 * row, 3, matrix[row].begin());
  return matrix;
}

class WrenchAggregationLogic {
public:
  const char *configure(const Parameters &parameters) {
    if (active_)
      return "Deactivate before configuring";
    if (!finite_vector(parameters.left.ankle_to_origin, 3) ||
        !finite_vector(parameters.right.ankle_to_origin, 3))
      return "Each ankle-to-plate offset requires three finite values";
    if (!finite_vector(parameters.left.linear_transform, 9) ||
        !finite_vector(parameters.right.linear_transform, 9) ||
        !finite_vector(parameters.symmetric_transform, 9))
      return "Each row-major transformation requires nine finite values";
    if (!std::isfinite(parameters.left.gain) ||
        !std::isfinite(parameters.right.gain) ||
        !std::isfinite(parameters.ankle_gain))
      return "Wrench gains must be finite";
    if (!std::isfinite(parameters.max_sample_age_s) ||
        parameters.max_sample_age_s <= 0.)
      return "Sample age must be positive and finite";

    if (!std::isfinite(parameters.max_sample_skew_s) ||
        parameters.max_sample_skew_s < 0.)
      return "Sample skew must be nonnegative and finite";

    Configuration next;
    std::copy_n(parameters.left.ankle_to_origin.begin(), 3,
                next.left.ankle_to_origin.begin());
    std::copy_n(parameters.right.ankle_to_origin.begin(), 3,
                next.right.ankle_to_origin.begin());
    next.left.linear_transform =
        matrix_from_values(parameters.left.linear_transform);
    next.right.linear_transform =
        matrix_from_values(parameters.right.linear_transform);
    next.left.gain = parameters.left.gain;
    next.right.gain = parameters.right.gain;
    next.symmetric_transform =
        matrix_from_values(parameters.symmetric_transform);
    next.ankle_gain = parameters.ankle_gain;
    next.same_transformation = parameters.same_transformation;
    next.same_scaling = parameters.same_scaling;
    next.custom_transformation = parameters.custom_transformation;
    next.matrix_transformation = parameters.matrix_transformation;
    next.scaling = parameters.scaling;
    next.max_sample_age_s = parameters.max_sample_age_s;
    next.max_sample_skew_s = parameters.max_sample_skew_s;
    configuration_ = next;
    configured_ = true;
    return nullptr;
  }

  const Configuration &configuration() const { return configuration_; }

  const char *validate_sample(const WrenchSample &sample, double time_s) const {
    if (!sample.valid || !finite(sample.wrench))
      return "Wrench sample is unavailable or nonfinite";
    if (!std::isfinite(sample.sample_time_s) || sample.sample_time_s < 0. ||
        sample.sample_time_s > time_s + TimestampToleranceS)
      return "Wrench timestamp is invalid or future-dated";
    if (time_s - sample.sample_time_s >
        configuration_.max_sample_age_s + TimestampToleranceS)
      return "Wrench sample is stale";
    return nullptr;
  }

  const char *validate_inputs(const Inputs &inputs, double time_s) const {
    if (!configured_)
      return "Logic has not been configured";
    if (!std::isfinite(time_s) || time_s < 0.)
      return "Invalid controller time";
    if (const auto *error = validate_sample(inputs.left, time_s))
      return error;
    if (const auto *error = validate_sample(inputs.right, time_s))
      return error;
    if (std::abs(inputs.left.sample_time_s - inputs.right.sample_time_s) >
        configuration_.max_sample_skew_s + TimestampToleranceS)
      return "Plate sample timestamps differ beyond the configured limit";
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

  void deactivate() { active_ = false; }

  UpdateResult update(const Inputs &inputs, double time_s,
                      double period_s) const {
    if (!active_)
      return {{}, "Logic is inactive"};
    if (!std::isfinite(period_s) || period_s < 0.)
      return {{}, "Invalid update period"};
    if (const auto *error = validate_inputs(inputs, time_s))
      return {{}, error};
    auto output = compute_wrench(inputs);
    if (const auto *error = validate_sample(output, time_s))
      return {{}, error};
    return {output, nullptr};
  }

private:
  WrenchSample compute_wrench(const Inputs &inputs) const {
    Wrench left = inputs.left.wrench;
    Wrench right = inputs.right.wrench;
    // Transport delivery can differ between plates. Report the older source
    // time so downstream age checks never make the combined wrench look newer.
    const double sample_time_s =
        std::min(inputs.left.sample_time_s, inputs.right.sample_time_s);

    if (configuration_.custom_transformation)
      wrench_transform::custom_transformation(left, right, sample_time_s);

    if (configuration_.matrix_transformation) {
      const auto &left_matrix = configuration_.same_transformation
                                    ? configuration_.symmetric_transform
                                    : configuration_.left.linear_transform;
      const auto &right_matrix = configuration_.same_transformation
                                     ? configuration_.symmetric_transform
                                     : configuration_.right.linear_transform;
      wrench_transform::apply_matrix_transform(left, left_matrix);
      wrench_transform::apply_matrix_transform(right, right_matrix);
    }
    if (configuration_.scaling) {
      const double left_gain = configuration_.same_scaling
                                   ? configuration_.ankle_gain
                                   : configuration_.left.gain;
      const double right_gain = configuration_.same_scaling
                                    ? configuration_.ankle_gain
                                    : configuration_.right.gain;
      wrench_transform::apply_gain(left, left_gain);
      wrench_transform::apply_gain(right, right_gain);
    }

    WrenchSample output;
    // Matrices are user operations on physical components, not frame changes.
    // Arms and output use physical axes attached to the foot platform, with
    // their origin at the ankle midpoint; these arms therefore stay constant.
    wrench_transform::plate_to_ankle(output.wrench, left, right,
                                     configuration_.left.ankle_to_origin,
                                     configuration_.right.ankle_to_origin);
    output.sample_time_s = sample_time_s;
    output.valid = true;
    return output;
  }

  Configuration configuration_;
  bool configured_ = false;
  bool active_ = false;
};
} // namespace riser_wrench_aggregation::logic
