#pragma once

#include <array>
#include <initializer_list>

namespace wrench_transform {
using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<Vector3, 3>;
inline constexpr Matrix3 Identity{{{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}}};

struct Wrench {
  Vector3 force{};
  Vector3 torque{};
};

inline Vector3 cross_product(const Vector3 &v, const Vector3 &w) {
  return {v[1] * w[2] - v[2] * w[1], v[2] * w[0] - v[0] * w[2],
          v[0] * w[1] - v[1] * w[0]};
}

inline Vector3 sum(std::initializer_list<Vector3> addends) {
  Vector3 result{};
  for (const auto &addend : addends)
    for (std::size_t dimension = 0; dimension < result.size(); ++dimension)
      result[dimension] += addend[dimension];
  return result;
}

inline Vector3 matrix_x_vector(const Vector3 &v, const Matrix3 &matrix) {
  Vector3 result{};
  for (std::size_t row = 0; row < result.size(); ++row)
    for (std::size_t col = 0; col < v.size(); ++col)
      result[row] += matrix[row][col] * v[col];
  return result;
}

// All vectors must use common axes. The arms point from ankle to plate origin.
inline void plate_to_ankle(Wrench &ankle, const Wrench &left,
                           const Wrench &right, const Vector3 &ankle_to_left,
                           const Vector3 &ankle_to_right) {
  ankle.force = sum({left.force, right.force});
  ankle.torque = sum({cross_product(ankle_to_left, left.force),
                      cross_product(ankle_to_right, right.force), left.torque,
                      right.torque});
}

inline void apply_matrix_transform(Wrench &wrench, const Matrix3 &transform) {
  wrench.force = matrix_x_vector(wrench.force, transform);
  wrench.torque = matrix_x_vector(wrench.torque, transform);
}

inline void apply_gain(Wrench &wrench, double gain) {
  for (std::size_t i = 0; i < wrench.force.size(); ++i) {
    wrench.force[i] *= gain;
    wrench.torque[i] *= gain;
  }
}

// Experiment hook, applied before the configured matrices and gains.
inline void custom_transformation(Wrench & /*left*/, Wrench & /*right*/,
                                  double /*sample_time_s*/) {}

} // namespace wrench_transform
