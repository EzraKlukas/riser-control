#pragma once
#include "profile.hpp"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <sstream>

namespace riser_motion {
using JogClock = std::chrono::steady_clock;
inline constexpr auto kJogRequestLifetime = std::chrono::milliseconds(500);
inline constexpr auto kJogFutureSkew = std::chrono::milliseconds(5);

inline std::string jog_stamp_error(std::chrono::nanoseconds age, int64_t stamp,
                                   int64_t latest_stamp) {
  if (age < -kJogFutureSkew || age > kJogRequestLifetime) {
    const auto age_ms = std::chrono::duration<double, std::milli>(age).count();
    return std::string(age.count() < 0 ? "Jog stamp is in the future"
                                       : "Jog stamp expired") +
           " (host ROS time minus client stamp is " + std::to_string(age_ms) +
           " ms, allowed " + std::to_string(-kJogFutureSkew.count()) + " to " +
           std::to_string(kJogRequestLifetime.count()) +
           " ms). Check host and client clocks.";
  }
  if (stamp <= latest_stamp)
    return "Jog stamp must be newer than the last accepted jog or cancel";
  return {};
}

inline std::string jog_limits_error(const Settings &s, double speed_cap,
                                    double duration_cap) {
  const char *reason = profile_error(s);
  if (std::isfinite(s.speed) && std::abs(s.speed) > speed_cap)
    reason = "Velocity cap exceeded";
  else if (std::isfinite(s.duration) && s.duration > duration_cap)
    reason = "Duration cap exceeded";
  if (!reason)
    return {};
  std::ostringstream message;
  message << reason << " (requested " << s.speed << " units/s for " << s.duration
          << " s. Configured limits are " << speed_cap << " units/s, " << duration_cap
          << " s, " << s.ramp << " s per ramp, " << s.max_acceleration
          << " units/s^2, " << s.max_travel << " units)";
  return message.str();
}

struct JogRequest {
  uint64_t id = 0, activation = 0;
  int64_t token = 0;
  double velocity = 0, duration = 0;
  JogClock::time_point expires;
  bool cancel = false;
  bool trajectory = false;
};

class JogProfile {
  static constexpr auto HeartbeatTimeout = std::chrono::milliseconds(600);
  uint64_t consumed_request_ = 0;
  JogRequest active_;
  JogClock::time_point started_, last_heartbeat_;
  double previous_position_ = 0, distance_ = 0, simulated_elapsed_ = 0;

public:
  bool running() const { return active_.id != 0; }
  void stop() { active_.id = 0; }

  double update(const Settings &caps, uint64_t activation,
                const JogRequest &request, int64_t heartbeat,
                JogClock::time_point heartbeat_time, JogClock::time_point now,
                double dt, const MotorSample &sample, double negative_limit,
                double positive_limit, bool simulation_time = false) {
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
        distance_ = simulated_elapsed_ = 0;
      }
    }

    if (!running())
      return 0;
    if (heartbeat == active_.token && heartbeat_time > last_heartbeat_)
      last_heartbeat_ = heartbeat_time;

    const double wall_elapsed =
        std::chrono::duration<double>(now - started_).count();
    const double elapsed = simulation_time ? simulated_elapsed_ : wall_elapsed;
    const double travelled =
        distance_ + std::abs(sample.position - previous_position_);
    if (!healthy || now - last_heartbeat_ >= HeartbeatTimeout ||
        elapsed >= active_.duration ||
        wall_elapsed > 4 * active_.duration + 2 ||
        (active_.velocity < 0 && negative_limit != 0) ||
        (active_.velocity > 0 && positive_limit != 0) ||
        (!caps.position_envelope &&
         (travelled >= caps.max_travel ||
          std::abs(sample.velocity) >
              3.0 * std::abs(active_.velocity) + caps.feedback_speed_tolerance))) {
      stop();
      return 0;
    }

    previous_position_ = sample.position;
    distance_ = travelled;
    simulated_elapsed_ += dt;
    // The requested duration includes both ends of the velocity ramp.
    const double ramp_fraction =
        std::min(elapsed, active_.duration - elapsed) / caps.ramp;
    const double rate = active_.velocity * std::min(1., ramp_fraction);
    return caps.position_envelope
               ? caps.position_envelope->limit_velocity(sample.position, rate, dt)
               : rate;
  }
};
} // namespace riser_motion
