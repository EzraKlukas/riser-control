#pragma once
#include <array>
#include <cstdint>

namespace riser_hardware {
// A mapping implementation must define its branch and admissible joint domain.
// q uses metres/radians; s uses actuator displacement in metres (rotary drives
// convert through measured transmission calibration below this boundary).
struct ActuatorState {
  std::array<double, 4> displacement_m;
  std::array<double, 4> velocity_m_s;
  std::array<double, 4> force_n;
};
class ActuatorMapping {
public:
  virtual ~ActuatorMapping() = default;
  virtual bool forward(const std::array<double, 4> &q,
                       std::array<double, 4> &s) const = 0;
  virtual bool inverse(const std::array<double, 4> &s,
                       const std::array<double, 4> &seed_q,
                       std::array<double, 4> &q) const = 0;
  // Row-major ds_i/dq_j: velocity_s=J*velocity_q;
  // effort_q=transpose(J)*force_s.
  virtual bool jacobian(const std::array<double, 4> &q,
                        std::array<double, 16> &J) const = 0;
};
// Transport contract: configuration/lifecycle outside read/update/write;
// cyclic receive/send must not allocate, log, sleep, or run a second bus loop.
// Raw encoder count conversion, homing offsets, signs, limits, validity, drive
// states and PDO layouts require hardware evidence. No values are inferred
// here.
struct DriveCalibration {
  double counts_per_actuator_m;
  // Measured encoder reading at the CAD reference pose, after homing.
  // counts = reference_counts + direction * counts_per_actuator_m * s.
  std::int64_t reference_counts;
  double direction;
};
} // namespace riser_hardware
