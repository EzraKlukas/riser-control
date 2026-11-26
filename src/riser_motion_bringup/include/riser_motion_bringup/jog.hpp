#pragma once
#include "profile.hpp"
#include <algorithm>
#include <chrono>
#include <cstdint>

namespace riser_motion_bringup {
using JogClock = std::chrono::steady_clock;
struct JogRequest {
  uint64_t id = 0, activation = 0;
  int64_t token = 0;
  double velocity = 0, duration = 0;
  JogClock::time_point expires;
  bool cancel = false;
};

class JogProfile {
  static constexpr auto HeartbeatTimeout = std::chrono::milliseconds(600);
  uint64_t consumed_request_ = 0;
  JogRequest active_;
  JogClock::time_point started_, last_heartbeat_;
  double previous_position_ = 0, distance_ = 0;

public:
  bool running() const { return active_.id != 0; }
  void stop() { active_.id = 0; }

  double update(const Settings &caps, uint64_t activation,
                const JogRequest &request, int64_t heartbeat,
                JogClock::time_point heartbeat_time, JogClock::time_point now,
                double dt, const MotorSample &sample, double negative_limit,
                double positive_limit) {
    const bool healthy = sample.ready == 2 && std::isfinite(sample.position) &&
                         std::isfinite(sample.velocity) && dt > 0 &&
                         dt <= caps.max_period;

    if (request.id != consumed_request_) {
      consumed_request_ = request.id;
      if (request.cancel) {
        stop();
      } else if (!running() && healthy && request.activation == activation &&
                 now <= request.expires) {
        // The service has already checked speed, duration and ramp bounds.
        active_ = request;
        started_ = last_heartbeat_ = now;
        previous_position_ = sample.position;
        distance_ = 0;
      }
    }

    if (!running())
      return 0;
    if (heartbeat == active_.token && heartbeat_time > last_heartbeat_)
      last_heartbeat_ = heartbeat_time;

    const double elapsed =
        std::chrono::duration<double>(now - started_).count();
    const double travelled =
        distance_ + std::abs(sample.position - previous_position_);
    if (!healthy || now - last_heartbeat_ >= HeartbeatTimeout ||
        elapsed >= active_.duration || travelled >= caps.max_travel ||
        (active_.velocity < 0 && negative_limit != 0) ||
        (active_.velocity > 0 && positive_limit != 0) ||
        std::abs(sample.velocity) >
            3.0 * std::abs(active_.velocity) + kSpeedToleranceMps) {
      stop();
      return 0;
    }

    previous_position_ = sample.position;
    distance_ = travelled;
    // The requested duration includes both ends of the velocity ramp.
    const double ramp_fraction =
        std::min(elapsed, active_.duration - elapsed) / caps.ramp;
    return active_.velocity * std::min(1., ramp_fraction);
  }
};
} // namespace riser_motion_bringup
