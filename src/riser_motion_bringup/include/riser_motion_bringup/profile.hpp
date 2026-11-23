#pragma once
#include <cmath>
#include <optional>
#include <string>
namespace riser_motion_bringup {
struct Settings {
  std::string profile = "ramp";
  double speed = 0.05, duration = 4, ramp = 1, max_travel = 0.2,
         max_acceleration = 0.1;
  double max_period = 0.005;
};
inline bool valid(const Settings &s) {
  const bool shape =
      s.profile == "ramp" || s.profile == "sine" || s.profile == "triangle";
  const double acceleration =
      s.profile == "sine" ? std::abs(s.speed) * 6.283185307179586 / s.duration
      : s.profile == "triangle" ? 2 * std::abs(s.speed) / s.duration
                                : std::abs(s.speed) / s.ramp;
  return shape && std::isfinite(s.speed) && std::isfinite(s.duration) &&
         s.duration > 0 && std::isfinite(s.ramp) && s.ramp > 0 &&
         s.ramp * 2 <= s.duration && std::isfinite(s.max_travel) &&
         s.max_travel > 0 && std::isfinite(s.max_acceleration) &&
         s.max_acceleration > 0 && acceleration <= s.max_acceleration &&
         std::isfinite(s.max_period) && s.max_period > 0 &&
         s.max_period < s.ramp &&
         std::abs(s.speed) * s.duration <= s.max_travel;
}
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
        std::abs(measured_velocity) > std::abs(settings_.speed) * 1.5 + 0.01) {
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
    return profile_.update(dt, sample.position, sample.velocity, sample.ready == 2);
  }
};
} // namespace riser_motion_bringup
