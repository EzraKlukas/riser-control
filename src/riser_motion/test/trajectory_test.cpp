#include "riser_motion/trajectory.hpp"
#include <cassert>

int main() {
  using namespace riser_motion;
  const Settings limits{"ramp", 0., 2., 0.25, 0.05, 0.12, 0.005};
  const auto parse = [&](const char *csv, bool position) {
    return parse_trajectory(csv, position, limits, 0.03, 10.);
  };
  for (const char *invalid : {
           "time_s,velocity\n0,0\n1,0.01\n1,0",
           "time_s,velocity\n0,0\n1,nan\n2,0",
           "time_s,velocity\n0,0\n1,0.031\n2,0",
           "time_s,velocity\n0,0\n0.01,0.02\n2,0",
           "time_s,velocity\n0,0.001\n1,0",
           "time_s,velocity\n0,0\n1,0.01",
           "time_s,velocity\n0,0\n1,0.02\n9,0.02\n10,0",
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
}
