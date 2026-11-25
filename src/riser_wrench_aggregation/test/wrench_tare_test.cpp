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

  for (const auto side : {PlateMode::Left, PlateMode::Right}) {
    WrenchAggregationLogic single;
    Parameters config;
    config.plate_mode = side;
    CHECK(single.configure(config) == nullptr);
    const bool left = side == PlateMode::Left;
    const auto &offset = left ? single.configuration().left.ankle_to_origin
                              : single.configuration().right.ankle_to_origin;
    CHECK(offset[0] == 0.);
    CHECK(single.activate() == nullptr);
    Inputs inputs;
    auto &plate = left ? inputs.left : inputs.right;
    plate = {{{5., Unavailable, 100.}, {2., Unavailable, Unavailable}}, true};
    auto sample = single.update(inputs, 0.1).output;
    CHECK(sample.valid && sample.wrench.force[0] == 5.);
    CHECK(sample.wrench.force[2] == 100.);
    CHECK(std::abs(sample.wrench.torque[0] - 12.16) < 1e-10);
    CHECK(std::isnan(sample.wrench.force[1]));
    CHECK(std::isnan(sample.wrench.torque[1]));
    CHECK(std::isnan(sample.wrench.torque[2]));
    plate.wrench.force[0] = Unavailable;
    CHECK(!single.update(inputs, 0.2).output.valid);
    plate.wrench.force[0] = 5.;
    plate.valid = false;
    CHECK(!single.update(inputs, 0.3).output.valid);

    single.deactivate();
    config.symmetric_transform[1] = 1.;
    CHECK(single.configure(config) != nullptr);
  }
}
