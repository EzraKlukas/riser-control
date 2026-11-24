#pragma once
#include "profile.hpp"
#include <chrono>

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
  MotorProfile profile_;
  uint64_t consumed_ = 0;
  int64_t token_ = 0;
  JogClock::time_point deadline_, lease_;

public:
  bool running() const { return token_ != 0; }
  void stop() { token_ = 0; }
  double update(const Settings &caps, uint64_t activation,
                const JogRequest &request, int64_t heartbeat,
                JogClock::time_point heartbeat_time, JogClock::time_point now,
                double dt, const MotorSample &sample, double negative_limit,
                double positive_limit) {
    if (token_ && heartbeat == token_)
      lease_ = heartbeat_time + std::chrono::milliseconds(300);
    if (now >= deadline_ || now >= lease_ || sample.ready != 2)
      stop();
    if (request.id != consumed_) {
      consumed_ = request.id;
      if (request.cancel)
        stop();
      else if (!token_ && request.activation == activation &&
               now <= request.expires && sample.ready == 2) {
        auto settings = caps;
        settings.speed = request.velocity;
        settings.duration = request.duration;
        if (profile_.start(settings, sample)) {
          token_ = request.token;
          deadline_ =
              now + std::chrono::duration_cast<JogClock::duration>(
                        std::chrono::duration<double>(settings.duration));
          lease_ = now + std::chrono::milliseconds(300);
        }
      }
    }
    if (!token_)
      return 0;
    auto target = profile_.update(dt, sample);
    if (!target || !std::isfinite(sample.position) ||
        !std::isfinite(sample.velocity) || !std::isfinite(dt) || dt <= 0 ||
        dt > caps.max_period || (*target < 0 && negative_limit != 0) ||
        (*target > 0 && positive_limit != 0)) {
      stop();
      return 0;
    }
    return *target;
  }
};
} // namespace riser_motion_bringup
