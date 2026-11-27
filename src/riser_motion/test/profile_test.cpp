#include <cstdlib>
#include <riser_motion/profile.hpp>
int main() {
  using namespace riser_motion;
  const Settings settings{"ramp", .02, 4, 1, .20, .08};
  Settings s = settings;
  if (!valid(s) || velocity(s, 0) != 0 || velocity(s, 4) != 0 ||
      velocity(s, 2) != s.speed)
    return 1;
  s.profile = "sine";
  if (std::abs(velocity(s, 1) - s.speed) > 1e-10 ||
      std::abs(velocity(s, 3) + s.speed) > 1e-10)
    return 2;
  Profile p;
  if (!p.start(s, 0) || p.update(0.001, 0, 0, true) == 0)
    return 3;
  if (p.update(0.01, 0, 0, true) != 0 || p.update(0.001, 0, 0, true) != 0)
    return 4;
  if (!p.start(s, 0) || p.update(0.001, NAN, 0, true) != 0)
    return 5;
  s.max_travel = 0.01;
  if (valid(s))
    return 6;
  s = settings;
  s.max_acceleration = 0.001;
  if (valid(s))
    return 7;
  s = settings;
  s.profile = "triangle";
  if (!valid(s) || velocity(s, 2) != s.speed)
    return 8;
  s = settings;
  s.profile = "sine";
  if (!p.start(s, 0))
    return 9;
  if (p.update(.001, .11, 0, true) == 0)
    return 10;
  if (p.update(.001, 0, 0, true) != 0)
    return 11;
  s = settings;
  if (!p.start(s, 0))
    return 12;
  for (int i = 0; i < 4001; ++i)
    p.update(.001, 0, 0, true);
  if (p.update(.001, 0, 0, true) != 0)
    return 13;
  return EXIT_SUCCESS;
}
