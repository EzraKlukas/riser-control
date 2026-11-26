#include <riser_motion_bringup/jog.hpp>

using namespace riser_motion_bringup;
using namespace std::chrono_literals;

int main() {
  Settings caps;
  caps.ramp = .25;
  caps.max_travel = .1;
  JogProfile jog;
  const auto start = JogClock::now();
  constexpr double speed_m_s = .02;
  JogRequest request{0, 1, 10, speed_m_s, 1., start};
  auto update = [&](auto elapsed, double negative = 0., double positive = 0.,
                    int64_t heartbeat = 0, MotorSample sample = {0., 0., 2.}) {
    const auto now = start + elapsed;
    return jog.update(caps, 1, request, heartbeat, now, now, .001, sample,
                      negative, positive);
  };
  for (double speed : {-speed_m_s, speed_m_s}) {
    request.velocity = speed;
    for (int negative : {0, 1})
      for (int positive : {0, 1}) {
        jog.stop();
        ++request.id;
        const bool blocked = speed < 0 ? negative : positive;
        update(0ms, negative, positive);
        if (update(125ms) != (blocked ? 0. : speed / 2))
          return 1;
      }
  }
  ++request.id;
  update(0ms);
  if (update(250ms, 0, 0, 10) != speed_m_s ||
      update(750ms, 0, 0, 10) != speed_m_s ||
      update(875ms, 0, 0, 10) != speed_m_s / 2 ||
      update(1s, 0, 0, 10) || jog.running())
    return 2;
  for (MotorSample sample :
       {MotorSample{.1, 0., 2.}, {0., .08, 2.}, {0., 0., 0.}, {NAN, 0., 2.}}) {
    ++request.id;
    update(0ms);
    if (update(1ms, 0, 0, 0, sample) || jog.running() || update(2ms))
      return 3;
  }
  ++request.id;
  if (update(1ms) || jog.running())
    return 4;
  ++request.id;
  update(0ms);
  if (!update(599ms) || update(600ms) || jog.running())
    return 5;
  ++request.id;
  update(0ms);
  ++request.id;
  request.cancel = true;
  if (update(1ms) || jog.running())
    return 6;
}
