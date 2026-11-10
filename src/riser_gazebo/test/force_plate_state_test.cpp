#include "riser_gazebo/force_plate_state.hpp"
#include <iostream>

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      std::cerr << "Failed at line " << __LINE__ << ": " << #condition         \
                << '\n';                                                       \
      return 1;                                                                \
    }                                                                          \
  } while (false)

int main() {
  riser_gazebo::ForcePlateState left, right;
  CHECK(left.values()[7] == 0.);
  for (std::size_t i = 0; i < 7; ++i)
    CHECK(std::isnan(left.values()[i]));
  left.read(1000000000, .1);
  CHECK(left.values()[7] == 0.);

  gz::msgs::Wrench message;
  message.mutable_header()->mutable_stamp()->set_sec(1);
  message.mutable_header()->mutable_stamp()->set_nsec(2000000);
  message.mutable_force()->set_x(1.);
  message.mutable_force()->set_y(-2.);
  message.mutable_force()->set_z(300.);
  message.mutable_torque()->set_x(-4.);
  message.mutable_torque()->set_y(5.);
  message.mutable_torque()->set_z(-6.);
  left.receive(message);
  left.read(1003000000, .1);
  const std::array<double, 6> expected{1., -2., 300., -4., 5., -6.};
  for (std::size_t i = 0; i < expected.size(); ++i)
    CHECK(left.values()[i] == expected[i]);
  CHECK(std::abs(left.values()[6] - 1.002) < 1e-12);
  CHECK(left.values()[7] == 1.);
  right.read(1003000000, .1);
  CHECK(right.values()[7] == 0.); // Independent sensor storage.

  left.read(1102000001, .1); // Stale samples cannot look like zero load.
  CHECK(left.values()[7] == 0.);
  CHECK(std::isnan(left.values()[2]));
  CHECK(std::abs(left.values()[6] - 1.002) < 1e-12);
  left.read(0, .1); // Clock reset: discard future-dated state.
  CHECK(left.values()[7] == 0.);
  left.read(1003000000, .1); // Time catching up cannot revive the old sample.
  CHECK(left.values()[7] == 0.);
  left.read(0, .1);
  message.mutable_header()->mutable_stamp()->set_sec(0);
  message.mutable_header()->mutable_stamp()->set_nsec(0);
  left.receive(message);
  left.read(0, .1);
  CHECK(left.values()[7] == 1.);

  message.mutable_force()->set_z(std::numeric_limits<double>::infinity());
  left.receive(message);
  left.read(0, .1);
  CHECK(left.values()[7] == 0.);
  for (std::size_t i = 0; i < 6; ++i)
    CHECK(std::isnan(left.values()[i]));
  message.mutable_force()->set_z(0.);
  message.clear_torque();
  left.receive(message);
  left.read(0, .1);
  CHECK(left.values()[7] == 0.);
  message.mutable_torque()->set_z(0.);
  message.mutable_header()->mutable_stamp()->set_nsec(1000000000);
  left.receive(message);
  left.read(0, .1);
  CHECK(left.values()[7] == 0.);
  CHECK(std::isnan(left.values()[6]));
  message.mutable_header()->mutable_stamp()->set_sec(
      std::numeric_limits<int64_t>::max());
  message.mutable_header()->mutable_stamp()->set_nsec(0);
  left.receive(message);
  left.read(0, .1);
  CHECK(left.values()[7] == 0.);
  message.clear_header();
  left.receive(message);
  left.read(0, .1);
  CHECK(left.values()[7] == 0.);
  std::cout << "Force-plate sample, isolation, freshness and invalid-data "
               "checks passed\n";
}
