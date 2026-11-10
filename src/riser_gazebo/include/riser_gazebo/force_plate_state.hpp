#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <gz/msgs/wrench.pb.h>
#include <limits>
#include <mutex>

namespace riser_gazebo {

// The transport thread owns pending_; only the hardware read thread writes
// values_. Exported handles retain stable addresses for the component lifetime.
class ForcePlateState {
public:
  inline static constexpr std::array<const char *, 8> InterfaceNames{
      "force.x",  "force.y",  "force.z",     "torque.x",
      "torque.y", "torque.z", "sample_time", "valid"};

  ForcePlateState() {
    values_.fill(std::numeric_limits<double>::quiet_NaN());
    values_[7] = 0.;
  }

  void receive(const gz::msgs::Wrench &message) {
    Sample sample;
    if (message.has_header() && message.header().has_stamp()) {
      const auto &stamp = message.header().stamp();
      constexpr int64_t billion = 1000000000;
      if (stamp.sec() >= 0 && stamp.nsec() >= 0 && stamp.nsec() < billion &&
          stamp.sec() <=
              (std::numeric_limits<int64_t>::max() - stamp.nsec()) / billion)
        sample.time_ns = stamp.sec() * billion + stamp.nsec();
    }
    sample.wrench = {message.force().x(),  message.force().y(),
                     message.force().z(),  message.torque().x(),
                     message.torque().y(), message.torque().z()};
    sample.valid =
        message.has_force() && message.has_torque() && sample.time_ns >= 0 &&
        std::all_of(sample.wrench.begin(), sample.wrench.end(),
                    [](double value) { return std::isfinite(value); });
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = sample;
  }

  void read(int64_t now_ns, double timeout_seconds) {
    if (last_read_time_ns_ >= 0 && now_ns < last_read_time_ns_)
      reset_pending_ = true;
    last_read_time_ns_ = now_ns;
    {
      // Never wait for the transport callback in the simulation/control loop.
      std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        // Do not let pre-reset samples become valid when time catches up.
        if (reset_pending_) {
          pending_ = Sample{};
          reset_pending_ = false;
        }
        current_ = pending_;
      }
    }
    const bool valid = !reset_pending_ && current_.valid &&
                       now_ns >= current_.time_ns &&
                       (now_ns - current_.time_ns) * 1e-9 <= timeout_seconds;
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < 6; ++i)
      values_[i] = valid ? current_.wrench[i] : unavailable;
    values_[6] = current_.time_ns >= 0 ? current_.time_ns * 1e-9 : unavailable;
    values_[7] = valid ? 1. : 0.;
  }

  std::array<double, 8> &values() { return values_; }

private:
  struct Sample {
    std::array<double, 6> wrench{};
    int64_t time_ns = -1;
    bool valid = false;
  };
  std::array<double, 8> values_;
  Sample pending_, current_;
  int64_t last_read_time_ns_ = -1;
  bool reset_pending_ = false;
  std::mutex mutex_;
};

} // namespace riser_gazebo
