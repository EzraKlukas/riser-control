#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
namespace riser_motion {
constexpr double kSpeedToleranceMps = 0.000016; // approximately 0.01 shaft rad/s

struct PositionEnvelope {
  double lower, upper;

  bool contains(double origin, double min_offset, double max_offset) const {
    return origin + min_offset >= lower && origin + max_offset <= upper;
  }

  double limit_velocity(double position, double rate, double dt) const {
    const double lower_rate = std::min(0., (lower - position) / dt);
    const double upper_rate = std::max(0., (upper - position) / dt);
    return std::clamp(rate, lower_rate, upper_rate);
  }
};

struct Settings {
  std::string profile = "ramp";
  double speed = 0, duration = 0, ramp = 0, max_travel = 0,
         max_acceleration = 0;
  double max_period = 0.005;
  // Footplate profiles use calibrated position instead of the cumulative travel budget.
  std::optional<PositionEnvelope> position_envelope = std::nullopt;
};
inline const char *profile_error(const Settings &s) {
  if (s.profile != "ramp" && s.profile != "sine" && s.profile != "triangle")
    return "Unknown profile shape";
  if (!std::isfinite(s.speed))
    return "Velocity must be finite";
  if (!std::isfinite(s.duration) || s.duration <= 0)
    return "Duration must be finite and positive";
  if (!std::isfinite(s.ramp) || s.ramp <= 0 || s.ramp * 2 > s.duration)
    return "Duration must allow two ramps of positive length";
  if (!std::isfinite(s.max_period) || s.max_period <= 0 || s.max_period >= s.ramp)
    return "Maximum control period must be positive and shorter than the ramp";
  if (s.position_envelope)
    return nullptr;
  const double acceleration =
      s.profile == "sine" ? std::abs(s.speed) * 6.283185307179586 / s.duration
      : s.profile == "triangle" ? 2 * std::abs(s.speed) / s.duration
                                : std::abs(s.speed) / s.ramp;
  if (!std::isfinite(s.max_acceleration) || s.max_acceleration <= 0 ||
      acceleration > s.max_acceleration)
    return "Acceleration cap exceeded";
  if (!std::isfinite(s.max_travel) || s.max_travel <= 0 ||
      std::abs(s.speed) * s.duration > s.max_travel)
    return "Travel cap exceeded";
  return nullptr;
}
inline bool valid(const Settings &s) { return profile_error(s) == nullptr; }
// Finite profiles with zero endpoints. Sine is one complete velocity cycle.
inline double velocity(const Settings &s, double t) {
  if (t <= 0 || t >= s.duration)
    return 0;
  if (s.profile == "sine")
    return s.speed * std::sin(6.283185307179586 * t / s.duration);
  if (s.profile == "triangle")
    return s.speed * (1 - std::abs(2 * t / s.duration - 1));
  return s.speed *
         std::fmin(1, std::fmin(t / s.ramp, (s.duration - t) / s.ramp));
}
class Profile {
  Settings settings_;
  double elapsed_ = 0, origin_ = 0, path_ = 0, previous_ = 0;
  bool stopped_ = true;

public:
  bool start(const Settings &settings, double position) {
    if (!valid(settings) || !std::isfinite(position))
      return false;
    settings_ = settings;
    elapsed_ = 0;
    origin_ = previous_ = position;
    path_ = 0;
    stopped_ = false;
    return true;
  }
  double update(double dt, double position, double measured_velocity,
                bool ready) {
    if (stopped_)
      return 0;
    path_ += std::abs(position - previous_);
    previous_ = position;
    if (!ready || !std::isfinite(dt) || dt <= 0 || dt > settings_.max_period ||
        !std::isfinite(position) || !std::isfinite(measured_velocity) ||
        std::abs(position - origin_) >= settings_.max_travel ||
        path_ >= settings_.max_travel ||
        std::abs(measured_velocity) >
            std::abs(settings_.speed) * 1.5 + kSpeedToleranceMps) {
      stopped_ = true;
      return 0;
    }
    elapsed_ += dt;
    if (elapsed_ >= settings_.duration) {
      stopped_ = true;
      return 0;
    }
    return velocity(settings_, elapsed_);
  }
};

struct MotorSample {
  double position;
  double velocity;
  double ready;
};

class MotorProfile {
  Profile profile_;
  bool waiting_ = true;
  double waiting_seconds_ = 0;

public:
  bool start(const Settings &settings, const MotorSample &sample) {
    waiting_ = true;
    waiting_seconds_ = 0;
    return (sample.ready == 1 || sample.ready == 2) &&
           std::isfinite(sample.velocity) &&
           profile_.start(settings, sample.position);
  }

  std::optional<double> update(double dt, const MotorSample &sample) {
    if (waiting_) {
      waiting_seconds_ += dt;
      if (!std::isfinite(waiting_seconds_) || dt <= 0 || waiting_seconds_ > 5 ||
          (sample.ready != 1 && sample.ready != 2))
        return std::nullopt;
      if (sample.ready != 2)
        return 0;
      waiting_ = false;
    }
    return profile_.update(dt, sample.position, sample.velocity,
                           sample.ready == 2);
  }
};
} // namespace riser_motion
