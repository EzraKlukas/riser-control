#pragma once

#include "jog.hpp"
#include <locale>
#include <stdexcept>
#include <vector>

namespace riser_motion {

struct Waypoint {
  double time, value;
};

// Motion CSV is deliberately separate from the ankle-wrench CSV format.
inline std::vector<Waypoint> parse_trajectory(
    const std::string &csv, bool position, const Settings &limits,
    double speed_cap, double duration_cap) {
  if (csv.size() > 256 * 1024)
    throw std::invalid_argument("Motion CSV exceeds 256 KiB");
  std::istringstream input(csv);
  input.imbue(std::locale::classic());
  std::string line;
  const auto read_line = [&] {
    if (!std::getline(input, line))
      return false;
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    return true;
  };
  if (!read_line() || line != (position ? "time_s,position" : "time_s,velocity"))
    throw std::invalid_argument("Expected time_s,position or time_s,velocity for the selected mode");
  std::vector<Waypoint> samples;
  double distance = 0;
  while (read_line()) {
    std::istringstream row(line);
    row.imbue(std::locale::classic());
    Waypoint next{};
    char comma = 0;
    if (!(row >> next.time >> comma >> next.value) || comma != ',' ||
        !(row >> std::ws).eof() || !std::isfinite(next.time) ||
        !std::isfinite(next.value) || next.time > duration_cap)
      throw std::invalid_argument("Invalid or out-of-budget CSV row " +
                                  std::to_string(samples.size() + 2));
    if (!position && std::abs(next.value) > speed_cap)
      throw std::invalid_argument("CSV velocity cap exceeded");
    if (samples.empty()) {
      if (next.time != 0 || next.value != 0)
        throw std::invalid_argument("CSV must start at time zero and value zero");
    } else {
      const auto previous = samples.back();
      const double dt = next.time - previous.time;
      const double change = next.value - previous.value;
      if (!(dt > 0))
        throw std::invalid_argument("CSV time must increase strictly");
      const double rate = change / dt;
      if (position) {
        if (1.5 * std::abs(rate) > speed_cap ||
            6 * std::abs(rate) / dt > limits.max_acceleration)
          throw std::invalid_argument("CSV position segment speed or acceleration exceeds cap");
        distance += std::abs(change);
      } else {
        if (std::abs(rate) > limits.max_acceleration)
          throw std::invalid_argument("CSV acceleration cap exceeded");
        distance += (std::abs(previous.value) + std::abs(next.value)) * dt / 2;
      }
      if (distance >= limits.max_travel)
        throw std::invalid_argument("CSV travel cap exceeded");
    }
    samples.push_back(next);
  }
  if (samples.size() < 2 || (!position && samples.back().value != 0))
    throw std::invalid_argument("CSV requires stationary start and end segments (zero endpoint velocities)");
  return samples;
}

struct Trajectory {
  Settings settings;
  std::vector<Waypoint> samples;
  double min_offset = 0, max_offset = 0;

  double duration() const {
    return samples.empty() ? settings.duration : samples.back().time;
  }

  double value(double time, bool position) const {
    if (samples.empty())
      return velocity(settings, time);
    const auto end = std::upper_bound(
        samples.begin(), samples.end(), time,
        [](double t, const Waypoint &p) { return t < p.time; });
    if (end == samples.end())
      return samples.back().value;
    if (end == samples.begin())
      return end->value;
    const auto &start = *(end - 1);
    double fraction = (time - start.time) / (end->time - start.time);
    if (position)
      fraction *= fraction * (3 - 2 * fraction); // Zero rate at each knot.
    return start.value + (end->value - start.value) * fraction;
  }
};

class TrajectoryPlayback {
  uint64_t id_ = 0;
  JogClock::time_point started_, last_heartbeat_;
  double elapsed_ = 0, origin_ = 0, previous_ = 0, distance_ = 0;

public:
  bool running() const { return id_ != 0; }
  void stop() { id_ = 0; }

  bool start(const JogRequest &request, JogClock::time_point now,
             const MotorSample &sample, double starting_speed,
             double position_origin) {
    if (now > request.expires || sample.ready != 2 ||
        !std::isfinite(sample.position) || !std::isfinite(sample.velocity) ||
        !std::isfinite(position_origin) ||
        std::abs(sample.velocity) > starting_speed)
      return false;
    id_ = request.id;
    elapsed_ = distance_ = 0;
    origin_ = position_origin;
    previous_ = sample.position;
    started_ = last_heartbeat_ = now;
    return true;
  }

  std::optional<double> update(
      const Trajectory &plan, bool position, const MotorSample &sample,
      double speed_cap, double dt, double reverse, double forward,
      int64_t token, int64_t beat, JogClock::time_point heartbeat,
      JogClock::time_point now, bool simulation_time = false) {
    if (token == beat && heartbeat > last_heartbeat_)
      last_heartbeat_ = heartbeat;
    distance_ += std::abs(sample.position - previous_);
    previous_ = sample.position;
    if (sample.ready != 2 || !std::isfinite(sample.position) ||
        !std::isfinite(sample.velocity) || !(dt > 0) ||
        dt > plan.settings.max_period || distance_ >= plan.settings.max_travel ||
        std::abs(sample.velocity) > speed_cap * 1.5 ||
        now - last_heartbeat_ >= std::chrono::milliseconds(600) ||
        now - started_ > std::chrono::duration<double>(
            simulation_time ? 4 * plan.duration() + 2 : plan.duration() + 1)) {
      stop();
      return std::nullopt;
    }
    const double value = plan.value(elapsed_, position);
    const double target = position ? origin_ + value : value;
    const double direction = position ? target - sample.position : target;
    if ((direction > 0 && forward != 0) || (direction < 0 && reverse != 0)) {
      stop();
      return std::nullopt;
    }
    if (elapsed_ >= plan.duration())
      stop();
    elapsed_ += dt;
    return target;
  }
};
} // namespace riser_motion
