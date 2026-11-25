#pragma once

#include "riser_wrench_aggregation/torque_perturbation.hpp"
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

struct PlateSample {
  Wrench wrench{};
  bool valid = false;
};

struct Inputs {
  PlateSample left;
  PlateSample right;
};

// Variable-length parameter values are checked before conversion to fixed
// sizes.
struct PlateParameters {
  std::vector<double> ankle_to_origin;
  std::vector<double> linear_transform{1., 0., 0., 0., 1., 0., 0., 0., 1.};
  double gain = 1.;
};

enum class PlateMode { Both, Left, Right };

struct Parameters {
  PlateMode plate_mode = PlateMode::Both;
  std::vector<double> single_plate_ankle_shift_left{0.127, 0., 0.};
  PlateParameters left{{0.127, 0.1016, 0.08509}};
  PlateParameters right{{-0.127, 0.1016, 0.08509}};
  std::vector<double> symmetric_transform{1., 0., 0., 0., 1., 0., 0., 0., 1.};
  double ankle_gain = 1.;
  bool same_transformation = true;
  bool same_scaling = true;
  bool custom_transformation = true;
  bool matrix_transformation = true;
  bool scaling = true;
};

struct PlateConfiguration {
  Vector3 ankle_to_origin{};
  Matrix3 linear_transform = wrench_transform::Identity;
  double gain = 1.;
};

struct Configuration {
  PlateMode plate_mode = PlateMode::Both;
  PlateConfiguration left, right;
  Matrix3 symmetric_transform = wrench_transform::Identity;
  double ankle_gain = 1.;
  bool same_transformation = true;
  bool same_scaling = true;
  bool custom_transformation = true;
  bool matrix_transformation = true;
  bool scaling = true;
};

enum class Capture { None, EmptyTare, StandingTare, Mass };
enum class CaptureResult {
  Pending,
  Success,
  InvalidSample,
  NeedsEmptyTare,
  PlateNotFlat,
  NoLoad,
  Cancelled
};

struct UpdateResult {
  WrenchSample output;
  const char *error = nullptr;
  CaptureResult capture = CaptureResult::Pending;
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
  const char *configure(const Parameters &parameters,
                        PerturbationParameters perturbation = {}) {
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
    if (const auto *error = perturbation_.configure(std::move(perturbation)))
      return error;
    Configuration next;
    std::copy_n(parameters.left.ankle_to_origin.begin(), 3,
                next.left.ankle_to_origin.begin());
    std::copy_n(parameters.right.ankle_to_origin.begin(), 3,
                next.right.ankle_to_origin.begin());
    next.plate_mode = parameters.plate_mode;
    if (next.plate_mode != PlateMode::Both) {
      if (!finite_vector(parameters.single_plate_ankle_shift_left, 3))
        return "Single-plate ankle shift needs three finite metres";
      const bool left = next.plate_mode == PlateMode::Left;
      auto &plate = left ? next.left : next.right;
      // Shift the central ankle origin toward the selected plate. The moment
      // arm points the opposite way, from the new ankle to the plate origin.
      for (std::size_t i = 0; i < 3; ++i)
        plate.ankle_to_origin[i] -=
            (left ? 1. : -1.) * parameters.single_plate_ankle_shift_left[i];

      const auto &matrix = parameters.same_transformation
                               ? parameters.symmetric_transform
                               : (left ? parameters.left.linear_transform
                                       : parameters.right.linear_transform);
      if (parameters.matrix_transformation &&
          (matrix[1] != 0 || matrix[7] != 0 || matrix[2] != 0))
        return "Single-plate transform mixes unmeasured force or torque components";
    }
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
    configuration_ = next;
    empty_tare_ = {};
    offset_ = {};
    has_empty_tare_ = false;
    subject_mass_kg_ = 0.;
    configured_ = true;
    return nullptr;
  }

  const Configuration &configuration() const { return configuration_; }

  const char *validate_inputs(const Inputs &inputs, double time_s) const {
    if (!configured_)
      return "Logic has not been configured";
    if (!std::isfinite(time_s) || time_s < 0.)
      return "Invalid read time";
    if (configuration_.plate_mode != PlateMode::Both) {
      const auto &plate = configuration_.plate_mode == PlateMode::Left
                              ? inputs.left : inputs.right;
      if (!plate.valid || !std::isfinite(plate.wrench.force[0]) ||
          !std::isfinite(plate.wrench.force[2]) ||
          !std::isfinite(plate.wrench.torque[0]))
        return "Selected plate needs valid Fx, Fz and torque about X";
      return nullptr;
    }
    if (!inputs.left.valid || !inputs.right.valid ||
        !finite(inputs.left.wrench) || !finite(inputs.right.wrench))
      return "Plate wrench is unavailable or nonfinite";
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

  void deactivate() {
    active_ = false;
    capture_ = {};
    reset_perturbation();
  }

  void begin_capture(Capture mode) {
    capture_ = {};
    capture_.mode = mode;
  }

  bool capture_active() const { return capture_.mode != Capture::None; }
  bool capture_needs_flat_plate() const {
    return capture_.mode == Capture::EmptyTare ||
           capture_.mode == Capture::Mass;
  }
  double subject_mass_kg() const { return subject_mass_kg_; }

  double &torque_x_before_perturbation() { return torque_x_before_nm_; }
  bool csv_playing() const { return perturbation_.csv_playing(); }

  template <class Register>
  void register_introspection(Register &&register_value) {
    register_value("torque_x_before_perturbation", &torque_x_before_nm_);
  }

  UpdateResult update(const Inputs &inputs, double read_time_s,
                      double foot_pitch_rad = Unavailable,
                      bool multisine_enabled = false, bool start_csv = false) {
    if (!active_)
      return invalid_update("Logic is inactive");
    if (const auto *error = validate_inputs(inputs, read_time_s))
      return invalid_update(error);
    auto output = compute_wrench(inputs, read_time_s);
    if (!finite(output.wrench))
      return invalid_update("Aggregated wrench is nonfinite");
    const auto capture = capture_active() ? collect(output, foot_pitch_rad)
                                          : CaptureResult::Pending;
    for (std::size_t i = 0; i < output.wrench.force.size(); ++i) {
      output.wrench.force[i] -= offset_.force[i];
      output.wrench.torque[i] -= offset_.torque[i];
    }
    torque_x_before_nm_ = output.wrench.torque[0];
    output.wrench.torque[0] +=
        perturbation_.update(read_time_s, multisine_enabled, start_csv);
    if (!finite(output.wrench)) {
      reset_perturbation();
      return {{}, "Final ankle wrench is nonfinite", capture};
    }
    if (configuration_.plate_mode != PlateMode::Both) {
      output.wrench.force[1] = Unavailable;
      output.wrench.torque[1] = Unavailable;
      output.wrench.torque[2] = Unavailable;
    }
    return {output, nullptr, capture};
  }

private:
  struct CaptureWindow {
    Capture mode = Capture::None;
    Wrench mean{};
    double first_time_s = 0.;
    double last_time_s = 0.;
    std::size_t samples = 0;
  };

  CaptureResult finish(CaptureResult result) {
    capture_ = {};
    return result;
  }

  UpdateResult invalid_update(const char *error) {
    reset_perturbation();
    return {{},
            error,
            capture_active() ? finish(CaptureResult::InvalidSample)
                             : CaptureResult::Pending};
  }

  void reset_perturbation() {
    torque_x_before_nm_ = Unavailable;
    perturbation_.reset();
  }

  CaptureResult collect(const WrenchSample &sample, double foot_pitch_rad) {
    constexpr double capture_duration_s = 1.;
    constexpr double max_read_gap_s = 0.1;
    constexpr double flat_tolerance_rad = 0.01;
    constexpr double gravity_m_s2 = 9.81;
    constexpr double minimum_load_n = 1.;

    if (capture_.mode == Capture::Mass && !has_empty_tare_)
      return finish(CaptureResult::NeedsEmptyTare);
    if (capture_needs_flat_plate() &&
        (!std::isfinite(foot_pitch_rad) ||
         std::abs(foot_pitch_rad) > flat_tolerance_rad))
      return finish(CaptureResult::PlateNotFlat);

    if (capture_.samples == 0) {
      capture_.first_time_s = sample.sample_time_s;
    } else {
      const double advance_s = sample.sample_time_s - capture_.last_time_s;
      if (advance_s < 0. || advance_s > max_read_gap_s)
        return finish(CaptureResult::InvalidSample);
      if (advance_s == 0.)
        return CaptureResult::Pending;
    }
    capture_.last_time_s = sample.sample_time_s;
    const double count = static_cast<double>(++capture_.samples);
    for (std::size_t i = 0; i < sample.wrench.force.size(); ++i) {
      capture_.mean.force[i] +=
          (sample.wrench.force[i] - capture_.mean.force[i]) / count;
      capture_.mean.torque[i] +=
          (sample.wrench.torque[i] - capture_.mean.torque[i]) / count;
    }
    if (sample.sample_time_s - capture_.first_time_s + TimestampToleranceS <
        capture_duration_s)
      return CaptureResult::Pending;
    if (!finite(capture_.mean))
      return finish(CaptureResult::InvalidSample);

    if (capture_.mode == Capture::Mass) {
      // The sensor can report the plate reaction or the applied vertical load.
      const double load_n =
          std::abs(capture_.mean.force[2] - empty_tare_.force[2]);
      if (!std::isfinite(load_n) || load_n < minimum_load_n)
        return finish(CaptureResult::NoLoad);
      subject_mass_kg_ = load_n / gravity_m_s2;
    } else {
      offset_ = capture_.mean;
      if (capture_.mode == Capture::EmptyTare) {
        empty_tare_ = offset_;
        has_empty_tare_ = true;
        subject_mass_kg_ = 0.;
      }
    }
    return finish(CaptureResult::Success);
  }

  WrenchSample compute_wrench(const Inputs &inputs, double read_time_s) const {
    Wrench left = inputs.left.wrench;
    Wrench right = inputs.right.wrench;
    if (configuration_.plate_mode != PlateMode::Both) {
      const bool use_left = configuration_.plate_mode == PlateMode::Left;
      auto plate = use_left ? left : right;
      const auto &config = use_left ? configuration_.left : configuration_.right;
      const auto &matrix = configuration_.matrix_transformation
          ? (configuration_.same_transformation ? configuration_.symmetric_transform
                                               : config.linear_transform)
          : wrench_transform::Identity;
      const double gain = configuration_.scaling
          ? (configuration_.same_scaling ? configuration_.ankle_gain : config.gain)
          : 1.;
      // Only these terms are identifiable. The selected ankle origin stays on
      // the pitch axis, and Fy is absent from the reduced moment calculation.
      const double fx = gain * (matrix[0][0] * plate.force[0] + matrix[0][2] * plate.force[2]);
      const double fz = gain * (matrix[2][0] * plate.force[0] + matrix[2][2] * plate.force[2]);
      const double tx = gain * matrix[0][0] * plate.torque[0] +
                        config.ankle_to_origin[1] * fz;
      return {{{fx, 0., fz}, {tx, 0., 0.}}, read_time_s, true};
    }
    if (configuration_.custom_transformation)
      wrench_transform::custom_transformation(left, right, read_time_s);

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
    wrench_transform::plate_to_ankle(output.wrench, left, right,
                                     configuration_.left.ankle_to_origin,
                                     configuration_.right.ankle_to_origin);
    output.sample_time_s = read_time_s;
    output.valid = true;
    return output;
  }

  Configuration configuration_;
  TorquePerturbation perturbation_;
  double torque_x_before_nm_ = Unavailable;
  CaptureWindow capture_;
  Wrench empty_tare_{};
  Wrench offset_{};
  double subject_mass_kg_ = 0.;
  bool has_empty_tare_ = false;
  bool configured_ = false;
  bool active_ = false;
};
} // namespace riser_wrench_aggregation::logic
