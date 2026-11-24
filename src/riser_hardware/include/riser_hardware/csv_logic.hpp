#pragma once
#include <cmath>
#include <cstdint>
#include <limits>

namespace riser_hardware {
// No automatic fault reset. A restart requires an operator to clear the drive
// fault.
inline uint16_t enable_word(uint16_t status) {
  // G5 keeps status bit 5 set outside "not ready" (I576 A-57).
  if ((status & 0x004f) == 0x0040)
    return 0x0006;
  switch (status & 0x006f) {
  case 0x0021:
    return 0x0007;
  case 0x0023:
  case 0x0027:
    return 0x000f;
  default:
    return 0x0006;
  }
}
// G5 ignores Quick Stop. Shutdown uses Pn735 (0x605B), exported as -1:
// dynamic braking during deceleration and while stopped (I576 A-3, A-58).
constexpr uint16_t kShutdownWord = 0x0006;
inline uint16_t control_word(uint16_t status, bool enable, bool fault) {
  return !fault && enable ? enable_word(status) : kShutdownWord;
}
inline bool operation_enabled(uint16_t status) {
  return (status & 0x006f) == 0x0027;
}
inline bool valid_scale(double value) {
  return std::isfinite(value) && value > 0;
}
inline bool safe_velocity(double command, double limit, double scale) {
  return std::isfinite(command) && std::abs(command) <= limit &&
         std::abs(command * scale) <= std::numeric_limits<int32_t>::max();
}
struct Feedback {
  bool healthy;
  double position, velocity;
  uint16_t status;
  int8_t mode;
};
class CsvGuard {
  // IgH's G5 DC acquisition has taken just over five seconds on the Jetson.
  static constexpr double kAcquisitionTimeout = 10;
  bool seen_ = false, enabled_ = false;
  unsigned fault_ = 0;
  double elapsed_ = 0, origin_ = 0;

public:
  void reset() {
    seen_ = enabled_ = false;
    fault_ = 0;
    elapsed_ = origin_ = 0;
  }
  void trip(unsigned reason = 1) {
    if (!fault_)
      fault_ = reason;
  }
  unsigned reason() const { return fault_; }
  bool fault() const { return fault_; }
  double observe(const Feedback &f, double dt, double cycle, double max_speed,
                 double max_travel, bool claimed = true) {
    elapsed_ += dt;
    if (!std::isfinite(dt) || dt <= 0 || (seen_ && dt > 5 * cycle))
      trip(2);
    if (!f.healthy) {
      if (seen_ || elapsed_ > kAcquisitionTimeout)
        trip(3);
      return 0;
    }
    if (!seen_)
      origin_ = f.position;
    seen_ = true;
    const bool enabled = operation_enabled(f.status);
    // G5 reports mode 0 while servo operation is disabled (I576 A-1-5).
    const bool valid_mode = f.mode == 9 || (!enabled && f.mode == 0);
    if ((claimed && enabled_ && !enabled) || (f.status & 0x006f) == 0x0007)
      trip(4);
    enabled_ = claimed && enabled;
    if (f.status & 8)
      trip(5);
    if (!std::isfinite(f.position) || !std::isfinite(f.velocity))
      trip(6);
    if (std::abs(f.position - origin_) >= max_travel)
      trip(7);
    if (std::abs(f.velocity) > max_speed)
      trip(8);
    if (!valid_mode)
      trip(9);
    return fault_ ? 0 : (enabled ? 2 : 1);
  }
};
} // namespace riser_hardware
