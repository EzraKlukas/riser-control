#include <cstdlib>
#include <initializer_list>
#include <riser_hardware/csv_logic.hpp>
#include <riser_hardware/g5_topology.hpp>
int main() {
  using namespace riser_hardware;
  if (enable_word(0x40) != 6 || enable_word(0x21) != 7 ||
      enable_word(0x23) != 15 || operation_enabled(0x08) ||
      !operation_enabled(0x27) || safe_velocity(NAN, 1, 1) ||
      safe_velocity(2, 1, 1) || safe_velocity(1, 1, 1e12) ||
      !safe_velocity(-0.5, 1, 100))
    return EXIT_FAILURE;
  std::vector<G5Identity> ring(4, {0x83, 4, 0x20001, 0, 0});
  std::vector<AxisAddress> addresses;
  for (unsigned i = 0; i < 4; ++i) {
    ring[i].serial = 10 + i;
    addresses.push_back({std::to_string(i), i, 0, 10 + i, i == 0});
  }
  validate_topology(addresses, ring);
  validate_topology(addresses, {ring.front()});
  for (auto &a : addresses)
    a.enabled = true;
  validate_topology(addresses, ring);
  std::swap(ring[1], ring[2]);
  try {
    validate_topology(addresses, ring);
    return 24;
  } catch (const std::runtime_error &) {
  }
  try {
    validate_topology(addresses, {ring.front()});
    return 25;
  } catch (const std::runtime_error &) {
  }
  CsvGuard guard;
  Feedback f{false, 0, 0, 0x40, 9};
  if (guard.observe(f, 1, .001, 1, 1) != 0 || guard.fault())
    return 2;
  f.healthy = true;
  if (guard.observe(f, .001, .001, 1, 1) != 1)
    return 3;
  f.status = 0x27;
  if (guard.observe(f, .001, .001, 1, 1) != 2)
    return 4;
  f.healthy = false;
  guard.observe(f, .001, .001, 1, 1);
  if (!guard.fault())
    return 5;
  f.healthy = true;
  if (guard.observe(f, .001, .001, 1, 1) != 0)
    return 6;
  guard.reset();
  f.position = 0;
  guard.observe(f, .001, .001, 1, 1);
  f.position = 1;
  if (guard.observe(f, .001, .001, 1, 1) != 0 || !guard.fault())
    return 7;
  guard.reset();
  f.position = 0;
  guard.observe(f, .001, .001, 1, 1);
  if (guard.observe(f, .006, .001, 1, 1) != 0 || !guard.fault())
    return 8;
  guard.reset();
  f.healthy = false;
  guard.observe(f, 11, .001, 1, 1);
  if (!guard.fault())
    return 9;
  guard.reset();
  f = {true, 0, 0, 0x27, 9};
  guard.observe(f, .001, .001, 1, 1);
  f.mode = 8;
  if (guard.observe(f, .001, .001, 1, 1) != 0 || !guard.fault())
    return 10;
  guard.reset();
  f = {true, 0, 2, 0x27, 9};
  if (guard.observe(f, .001, .001, 1, 1) != 0 || !guard.fault())
    return 11;
  guard.reset();
  f = {true, 0, 0, 0x27, 9};
  guard.observe(f, .001, .001, 1, 1);
  f.status = 0x23;
  if (guard.observe(f, .001, .001, 1, 1) != 0 || !guard.fault())
    return 12;
  // G5's disabled status is 0x60, with warning/remote bits possibly set.
  if (enable_word(0x60) != 6 || enable_word(0x2e0) != 6)
    return 13;
  for (uint16_t status :
       {uint16_t(0x60), uint16_t(0x21), uint16_t(0x23), uint16_t(0x27)}) {
    if (control_word(status, true, true) != 6 ||
        control_word(status, false, false) != 6)
      return 14;
  }
  if (control_word(0x27, true, false) != 15)
    return 15;
  guard.reset();
  f = {true, 0, 0, 0x0a70, 0};
  if (guard.observe(f, .001, .001, 1, 1, false) != 1 ||
      guard.observe(f, 6, .001, 1, 1, false) != 0 || !guard.fault())
    return 16;
  guard.reset();
  if (guard.observe(f, .001, .001, 1, 1, false) != 1)
    return 17;
  for (int cycle = 0; cycle < 6000; ++cycle) {
    if (guard.observe(f, .001, .001, 1, 1, false) != 1)
      return 18;
  }
  f.status = 0x27;
  if (guard.observe(f, .001, .001, 1, 1) != 0 || !guard.fault())
    return 19;
  guard.reset();
  f.mode = 9;
  if (guard.observe(f, .001, .001, 1, 1) != 2)
    return 20;
  f.status = 0x60;
  f.mode = 0;
  if (guard.observe(f, .001, .001, 1, 1, false) != 1)
    return 21;
  guard.reset();
  f.healthy = false;
  for (int cycle = 0; cycle < 5300; ++cycle)
    if (guard.observe(f, .001, .001, 1, 1, false) != 0 || guard.fault())
      return 22;
  f.healthy = true;
  if (guard.observe(f, .001, .001, 1, 1, false) != 1)
    return 23;
  return EXIT_SUCCESS;
}
