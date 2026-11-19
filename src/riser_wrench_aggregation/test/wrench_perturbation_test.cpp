#include "riser_wrench_aggregation/wrench_aggregation_logic.hpp"
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
  using namespace riser_wrench_aggregation::logic;
  WrenchAggregationLogic logic;
  PerturbationParameters setup;
  setup.multisine_peak_nm = 5.;
  setup.csv_sample_period_s = 0.25;
  setup.csv = {1., 8., 0.};
  CHECK(logic.configure({}, setup));
  setup.csv.front() = 0.;
  CHECK(!logic.configure({}, setup));
  CHECK(!logic.activate());

  const Inputs inputs{{{{0., 0., 0.}, {2., 3., 4.}}, true}, {{}, true}};
  CHECK(logic.update(inputs, 1.).output.wrench.torque[0] == 2.);
  CHECK(logic.update(inputs, 2., 0., false, true).output.wrench.torque[0] ==
        2.);
  CHECK(logic.csv_playing());
  const auto interpolated = logic.update(inputs, 2.125, 0.).output;
  CHECK(interpolated.wrench.torque == (Vector3{6., 3., 4.}));
  CHECK(logic.torque_x_before_perturbation() == 2.);
  CHECK(logic.update(inputs, 2.25, 0., false, true).output.wrench.torque[0] ==
        10.);
  CHECK(logic.update(inputs, 2.5, 0.).output.wrench.torque[0] == 2.);
  CHECK(!logic.csv_playing());

  logic.update(inputs, 3., 0., false, true);
  CHECK(!logic.update({}, 3.125).output.valid);
  CHECK(!logic.csv_playing());
  CHECK(std::isnan(logic.torque_x_before_perturbation()));

  logic.begin_capture(Capture::StandingTare);
  logic.update(inputs, 4., 0., true, true);
  bool noise_seen = false;
  UpdateResult result;
  for (int i = 1; i <= 20; ++i) {
    result = logic.update(inputs, 4. + .05 * i, 0., true);
    noise_seen |= std::abs(result.output.wrench.torque[0] -
                           logic.torque_x_before_perturbation()) > .1;
  }
  CHECK(noise_seen);
  CHECK(result.capture == CaptureResult::Success);
  CHECK(logic.torque_x_before_perturbation() == 0.);
  CHECK(std::abs(result.output.wrench.torque[0]) <= setup.multisine_peak_nm);
  CHECK(!logic.csv_playing());
  CHECK(logic.update(inputs, 5.5, 0., false).output.wrench.torque[0] == 0.);

  TorquePerturbation noise, combined;
  CHECK(!noise.configure(setup));
  CHECK(!combined.configure(setup));
  noise.update(10., true, false);
  combined.update(10., true, true);
  const double noise_nm = noise.update(10.25, true, false);
  const double combined_nm = combined.update(10.25, true, false);
  CHECK(std::abs(combined_nm - noise_nm - 8.) < 1e-12);
}
