#include "riser_motion/trajectory.hpp"
#include <cassert>
#include <iomanip>
#include <limits>

int main() {
  using namespace riser_motion;
  const Settings limits{"ramp", 0., 2., 0.25, 0.05, 0.12, 0.005};
  const auto parse = [&](const std::string &csv, bool position) {
    return parse_trajectory(csv, position, limits, 0.03, 60.);
  };
  for (const char *invalid : {
           "time_s,velocity\n0,0\n1,0.01\n1,0",
           "time_s,velocity\n0,0\n1,nan\n2,0",
           "time_s,velocity\n0,0\n1,0.031\n2,0",
           "time_s,velocity\n0,0\n0.01,0.02\n2,0",
           "time_s,velocity\n0,0.001\n1,0",
           "time_s,velocity\n0,0\n1,0.01",
           "time_s,velocity\n0,0\n1,0.02\n9,0.02\n10,0",
           "time_s,velocity\n0,0\n60.001,0",
       }) {
    try {
      parse(invalid, false);
      return 1;
    } catch (const std::invalid_argument &) {
    }
  }
  const Trajectory position{limits, parse("time_s,position\n0,0\n1,0.006", true)};
  const Trajectory velocity{limits,
                            parse("time_s,velocity\r\n0,0\r\n1,0.01\r\n2,0\r\n", false)};
  Settings footplate = limits;
  footplate.position_envelope = PositionEnvelope{-0.18, 0.18};
  std::ostringstream dense_csv;
  dense_csv << "time_s,velocity\r\n" << std::scientific
            << std::setprecision(std::numeric_limits<double>::max_digits10 - 1);
  for (int millisecond = 0; millisecond <= 60000; ++millisecond) {
    const double time = millisecond / 1000.;
    const double phase = (millisecond % 1000) / 1000.;
    const double rate = 0.5 * std::sin(6.283185307179586 * phase);
    dense_csv << time << ',' << rate << "          \r\n";
  }
  const auto dense = dense_csv.str();
  Trajectory oscillation{footplate, parse_trajectory(dense, false, footplate, 0., 60.)};
  oscillation.measure_position_offsets(false);
  const auto &envelope = *footplate.position_envelope;
  if (dense.size() <= 3 * 1024 * 1024 || oscillation.samples.size() != 60001 ||
      oscillation.duration() != 60. ||
      !envelope.contains(0., oscillation.min_offset, oscillation.max_offset) ||
      envelope.contains(0.04, oscillation.min_offset, oscillation.max_offset))
    return 1;

  Trajectory reversing{footplate, parse_trajectory(
      "time_s,velocity\n0,0\n0.1,1\n1.1,-1\n1.2,0", false, footplate, 0., 60.)};
  reversing.measure_position_offsets(false);
  if (std::abs(reversing.max_offset - 0.3) > 1e-9 ||
      envelope.contains(0., reversing.min_offset, reversing.max_offset))
    return 1;

  Trajectory steep{footplate, parse_trajectory(
      "time_s,position\n0,0\n0.001,0.18\n0.002,-0.18", true, footplate, 0., 60.)};
  steep.measure_position_offsets(true);
  if (!envelope.contains(0., steep.min_offset, steep.max_offset) ||
      envelope.contains(0.001, steep.min_offset, steep.max_offset))
    return 1;

  assert(parse("time_s,velocity\n0,0\n0.001,0.00001\n0.002,0", false).size() == 3);
  assert(std::abs(position.value(0.25, true) - 0.0009375) < 1e-9);
  assert(std::abs(velocity.value(0.25, false) - 0.0025) < 1e-9);
  const auto now = JogClock::now();
  JogRequest request;
  request.id = 1;
  request.token = 100;
  request.expires = now + kJogRequestLifetime;
  TrajectoryPlayback playback;
  MotorSample sample{0.023, 0., 2.};
  const double held = 0.024;
  assert(playback.start(request, now, sample, 0.001, held));
  assert(playback.update(position, true, sample, .03, .001, 0, 0,
                         request.token, 0, now, now) == held);
  assert(!playback.update(position, true, sample, .03, .001, 0, 1,
                          request.token, 0, now, now));
  assert(!playback.running());
  assert(playback.start(request, now, sample, 0.001, held));
  sample.ready = 0.;
  assert(!playback.update(position, true, sample, .03, .001, 0, 0,
                          request.token, 0, now, now));
  assert(!playback.running());

  MotorSample near_limit{0.1799, 0., 2.};
  if (!playback.start(request, now, near_limit, 0.0015, near_limit.position))
    return 1;
  const auto rate_at = [&](double angle) {
    near_limit.position = angle;
    return playback.update(oscillation, false, near_limit, 0., .004, 0, 0,
                           request.token, 0, now, now);
  };
  if (rate_at(.18) != 0. || rate_at(.18) != 0. || !playback.running())
    return 1;
  const auto near_limit_rate = rate_at(.17999);
  if (!near_limit_rate || std::abs(*near_limit_rate - .0025) > 1e-9 ||
      envelope.limit_velocity(-.18, -1., .001) != 0. ||
      envelope.limit_velocity(.18, -1., .001) != -1.)
    return 1;
}
