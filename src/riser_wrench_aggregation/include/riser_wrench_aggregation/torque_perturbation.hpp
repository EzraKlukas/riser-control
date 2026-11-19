#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

namespace riser_wrench_aggregation::logic {

struct PerturbationParameters {
  double multisine_peak_nm = 0.;
  double multisine_low_hz = 0.2;
  double multisine_high_hz = 2.;
  double csv_sample_period_s = 0.001;
  std::vector<double> csv;
};

class TorquePerturbation {
public:
  const char *configure(PerturbationParameters parameters) {
    if (!std::isfinite(parameters.multisine_peak_nm) ||
        parameters.multisine_peak_nm < 0. ||
        !std::isfinite(parameters.multisine_low_hz) ||
        !std::isfinite(parameters.multisine_high_hz) ||
        parameters.multisine_low_hz <= 0. ||
        parameters.multisine_high_hz <= parameters.multisine_low_hz)
      return "Multisine requires a nonnegative finite peak and 0 < low_hz < "
             "high_hz";
    if (!std::isfinite(parameters.csv_sample_period_s) ||
        parameters.csv_sample_period_s <= 0.)
      return "CSV sample period must be positive and finite";

    if (!parameters.csv.empty()) {
      if (parameters.csv.size() < 2 ||
          !std::isfinite((parameters.csv.size() - 1) *
                         parameters.csv_sample_period_s) ||
          parameters.csv.front() != 0. || parameters.csv.back() != 0. ||
          !std::all_of(parameters.csv.begin(), parameters.csv.end(),
                       [](double v) { return std::isfinite(v); }))
        return "CSV requires finite samples, a finite duration and zero "
               "endpoints";
    }
    for (std::size_t i = 0; i < frequencies_hz_.size(); ++i) {
      const double fraction =
          static_cast<double>(i) / (frequencies_hz_.size() - 1);
      frequencies_hz_[i] = parameters.multisine_low_hz +
                           fraction * (parameters.multisine_high_hz -
                                       parameters.multisine_low_hz);
    }
    settings_ = std::move(parameters);
    reset();
    return nullptr;
  }

  void reset() {
    last_read_.reset();
    csv_start_.reset();
    multisine_time_s_ = 0.;
    gain_ = 0.;
  }

  bool csv_playing() const { return csv_start_.has_value(); }

  double update(double read_time_s, bool multisine_enabled, bool start_csv) {
    if (last_read_ && read_time_s < *last_read_)
      reset();
    const double dt_s = last_read_ ? read_time_s - *last_read_ : 0.;
    last_read_ = read_time_s;

    if (start_csv && !settings_.csv.empty() && !csv_playing())
      csv_start_ = read_time_s;
    return multisine(dt_s, multisine_enabled) + csv_at(read_time_s);
  }

private:
  double multisine(double dt_s, bool enabled) {
    if (settings_.multisine_peak_nm == 0. || (!enabled && gain_ == 0.))
      return 0.;

    constexpr double ramp_s = 0.5;
    constexpr double two_pi = 6.283185307179586;
    gain_ = std::clamp(gain_ + (enabled ? dt_s : -dt_s) / ramp_s, 0., 1.);
    if (gain_ == 0.) {
      multisine_time_s_ = 0.;
      return 0.;
    }

    multisine_time_s_ += dt_s;
    double sum = 0.;
    for (double hz : frequencies_hz_)
      sum += std::sin(two_pi * hz * multisine_time_s_);
    return gain_ * (sum / frequencies_hz_.size()) * settings_.multisine_peak_nm;
  }

  double csv_at(double time_s) {
    if (!csv_start_)
      return 0.;
    const double index = (time_s - *csv_start_) / settings_.csv_sample_period_s;
    if (index >= settings_.csv.size() - 1) {
      csv_start_.reset();
      return 0.;
    }

    const auto sample = static_cast<std::size_t>(index);
    const double fraction = index - sample;
    return (1. - fraction) * settings_.csv[sample] +
           fraction * settings_.csv[sample + 1];
  }

  PerturbationParameters settings_;
  std::array<double, 5> frequencies_hz_{};
  std::optional<double> last_read_;
  std::optional<double> csv_start_;
  double multisine_time_s_ = 0.;
  double gain_ = 0.;
};

} // namespace riser_wrench_aggregation::logic
