#include "riser_wrench_aggregation/wrench_aggregation_logic.hpp"
#include <iostream>

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      std::cerr << "Failed at line " << __LINE__ << ": " << #condition << '\n'; \
      return 1;                                                                \
    }                                                                          \
  } while (false)

int main() {
  using namespace riser_wrench_aggregation::logic;
  WrenchAggregationLogic logic;
  CHECK(logic.configure({}) == nullptr);
  CHECK(logic.activate() == nullptr);

  const Wrench empty{{1., 2., -10.}, {4., 5., 6.}};
  const Wrench loaded{{1., 2., -10. - 70. * 9.81}, {18., 7., 9.}};
  const auto update = [&](const Wrench &wrench, double stamp, double pitch) {
    return logic.update({{wrench, true}, {{}, true}}, stamp, pitch);
  };
  const auto capture = [&](const Wrench &wrench, double start) {
    for (int i = 0; i < 20; ++i) {
      if (update(wrench, start + i * .05, 0.).capture != CaptureResult::Pending)
        return CaptureResult::InvalidSample;
    }
    // A repeated update with the same timestamp is not a new hardware read.
    if (update({}, start + 19 * .05, 0.).capture != CaptureResult::Pending)
      return CaptureResult::InvalidSample;
    return update(wrench, start + 1., 0.).capture;
  };
  const auto zero = [](const WrenchSample &sample) {
    return sample.valid && sample.wrench.force == Vector3{} &&
           sample.wrench.torque == Vector3{};
  };

  CHECK(update(loaded, 0.25, 0.).output.sample_time_s == 0.25);
  logic.begin_capture(Capture::Mass);
  CHECK(update(loaded, 1., 0.).capture == CaptureResult::NeedsEmptyTare);
  logic.begin_capture(Capture::EmptyTare);
  CHECK(update(empty, 1., .02).capture == CaptureResult::PlateNotFlat);
  logic.begin_capture(Capture::EmptyTare);
  CHECK(capture(empty, 1.) == CaptureResult::Success);
  CHECK(zero(update(empty, 2.05, 0.).output));
  CHECK(logic.subject_mass_kg() == 0.);

  logic.begin_capture(Capture::Mass);
  CHECK(capture(loaded, 3.) == CaptureResult::Success);
  CHECK(std::abs(logic.subject_mass_kg() - 70.) < 1e-10);
  logic.begin_capture(Capture::StandingTare);
  CHECK(capture(loaded, 5.) == CaptureResult::Success);
  CHECK(zero(update(loaded, 6.05, 0.).output));
  CHECK(std::abs(logic.subject_mass_kg() - 70.) < 1e-10);

  logic.begin_capture(Capture::StandingTare);
  CHECK(capture(loaded, 7.) == CaptureResult::Success);
  CHECK(zero(update(loaded, 8.05, 0.).output));
  logic.begin_capture(Capture::StandingTare);
  CHECK(update(loaded, 9., 0.).capture == CaptureResult::Pending);
  CHECK(logic.update({}, 9.05).capture == CaptureResult::InvalidSample);
  CHECK(zero(update(loaded, 9.1, 0.).output));

  logic.begin_capture(Capture::Mass);
  CHECK(capture(loaded, 10.) == CaptureResult::Success);
  CHECK(std::abs(logic.subject_mass_kg() - 70.) < 1e-10);
  logic.deactivate();
  CHECK(logic.activate() == nullptr);
  CHECK(zero(update(loaded, 11.05, 0.).output));
  logic.begin_capture(Capture::EmptyTare);
  CHECK(capture(empty, 12.) == CaptureResult::Success);
  CHECK(logic.subject_mass_kg() == 0.);
}
