#pragma once

#include <cstdint>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <istream>
#include <ostream>

namespace riser_gazebo {
// Written in PreUpdate, then read on the simulation thread during hardware I/O.
// simTime labels the end of the frame; step_ns is the actual physics interval.
struct PhysicsClock {
  int64_t time_ns = 0;
  int64_t step_ns = 0;
  bool paused = true;

  bool operator==(const PhysicsClock &other) const {
    return time_ns == other.time_ns && step_ns == other.step_ns &&
           paused == other.paused;
  }
};

inline std::ostream &operator<<(std::ostream &stream,
                                const PhysicsClock &clock) {
  return stream << clock.time_ns << ' ' << clock.step_ns << ' ' << clock.paused;
}
inline std::istream &operator>>(std::istream &stream, PhysicsClock &clock) {
  return stream >> clock.time_ns >> clock.step_ns >> clock.paused;
}

using PhysicsClockComponent =
    gz::sim::components::Component<PhysicsClock, class RiserPhysicsClockTag>;
GZ_SIM_REGISTER_COMPONENT("riser_gazebo.PhysicsClock", PhysicsClockComponent)
} // namespace riser_gazebo
