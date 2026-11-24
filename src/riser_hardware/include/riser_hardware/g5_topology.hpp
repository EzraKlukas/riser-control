#pragma once
#include <algorithm>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace riser_hardware {
struct G5Identity {
  uint32_t vendor, product, revision, alias, serial;
};
struct AxisAddress {
  std::string name;
  uint32_t ring, alias, serial;
  bool enabled;
};

// Non-real-time commissioning check. Excluded drives may be absent. Every
// drive actually on the ring must match one commissioned physical location.
inline void validate_topology(const std::vector<AxisAddress> &axes,
                              const std::vector<G5Identity> &ring) {
  std::set<uint32_t> assigned, serials;
  for (const auto &a : axes) {
    if (!assigned.insert(a.ring).second ||
        (a.serial && !serials.insert(a.serial).second))
      throw std::runtime_error("Duplicate ring position or serial: " + a.name);
    if (a.ring >= ring.size()) {
      if (a.enabled)
        throw std::runtime_error("Missing enabled drive: " + a.name);
      continue;
    }
    const auto &drive = ring[a.ring];
    if (drive.vendor != 0x83 || drive.product != 4 ||
        drive.revision != 0x20001 || drive.alias != a.alias ||
        (a.serial && drive.serial != a.serial) ||
        (!a.serial && (ring.size() != 1 || a.name != "bottom_motor")))
      throw std::runtime_error(
          "G5 identity mismatch or uncommissioned serial: " + a.name);
  }
  for (size_t position = 0; position < ring.size(); ++position)
    if (!assigned.count(position))
      throw std::runtime_error("Unassigned EtherCAT slave at " +
                               std::to_string(position));
}
} // namespace riser_hardware
