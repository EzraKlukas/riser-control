#include <riser_motion_bringup/jog.hpp>
using namespace riser_motion_bringup;
int main() {
  Settings s;
  JogProfile jog;
  JogRequest r;
  auto now = JogClock::now();
  r.activation = 1;
  r.token = 10;
  r.duration = 4;
  r.expires = now;
  double ready = 2;
  auto update = [&](double neg, double pos) {
    return jog.update(s, 1, r, 0, now, now, .001, {0, 0, ready}, neg, pos);
  };
  for (double sign : {-1., 1.}) {
    r.velocity = sign * .04;
    for (int neg = 0; neg <= 1; ++neg)
      for (int pos = 0; pos <= 1; ++pos) {
        jog.stop();
        ++r.id;
        const bool blocked = sign > 0 ? pos : neg;
        if ((update(neg, pos) * sign > 0) == blocked)
          return 1;
        if (blocked && update(0, 0) != 0)
          return 2;
      }
  }
  ++r.id;
  if (update(0, 0) <= 0)
    return 3;
  ++r.id;
  r.cancel = true;
  if (update(0, 0) != 0)
    return 4;
  ++r.id;
  r.cancel = false;
  now += std::chrono::seconds(1);
  if (update(0, 0) != 0)
    return 5;
  ++r.id;
  r.expires = now;
  if (update(0, 0) <= 0)
    return 6;
  now += std::chrono::milliseconds(301);
  if (update(0, 0) != 0)
    return 7;
  ++r.id;
  r.expires = now;
  if (update(0, 0) <= 0)
    return 8;
  ready = 0;
  if (update(0, 0) != 0)
    return 9;
  ready = 2;
  if (update(0, 0) != 0)
    return 10;
}
