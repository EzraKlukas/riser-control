#pragma once
#include <cstdint>
#include <riser_kinematics/lateral.hpp>
#include <riser_kinematics/mapping.hpp>

namespace riser_hardware {
// Geometry/kinematics are shared with simulation. Coordinates are independent
// [frame_pitch, torso_lateral, hip_lateral, foot_pitch], not the three public
// angular coordinates. Mapping velocity uses its diagonal ds_i/dq_i Jacobian.
using ActuatorMapping = riser_kinematics::Mapping;
using ActuatorCoordinates = riser_kinematics::Coordinates;
struct ActuatorState {
  ActuatorCoordinates displacement_m;
  ActuatorCoordinates velocity_m_s;
  ActuatorCoordinates force_n;
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
