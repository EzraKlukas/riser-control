#include "riser_kinematics/lateral.hpp"
#include "riser_kinematics/mapping.hpp"
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <new>
#include <string>

// Count allocations while repeatedly executing both successful and failing
// calls.
static std::atomic<std::size_t> allocations{0};
void *operator new(std::size_t n) {
  ++allocations;
  if (void *p = std::malloc(n ? n : 1))
    return p;
  throw std::bad_alloc();
}
void operator delete(void *p) noexcept { std::free(p); }
void operator delete(void *p, std::size_t) noexcept { std::free(p); }

using namespace riser_kinematics;
#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      std::cerr << __LINE__ << ": " #condition "\n";                           \
      return 1;                                                                \
    }                                                                          \
  } while (false)
static bool near(double a, double b, double tolerance = 1e-10) {
  return std::abs(a - b) <= tolerance;
}
int main(int argc, char **argv) {
  const auto geometry = canonical_geometry();
  const std::array<Domain, 4> domains{
      {{-.08, .08}, {-.05, .05}, {-.05, .05}, {-.08, .08}}};
  if (argc == 2 && std::string(argv[1]) == "--samples") {
    std::array<Axis, 4> axes;
    for (std::size_t i = 0; i < 4; ++i) {
      Domain domain;
      CHECK(std::cin >> domain.lower >> domain.upper);
      CHECK(axes[i].configure(geometry[i], domain) == Error::None);
    }
    std::size_t axis;
    double q, qdot;
    std::cout << std::setprecision(17);
    while (std::cin >> axis >> q >> qdot) {
      CHECK(axis < 4);
      const auto f = axes[axis].forward(q, qdot);
      CHECK(f);
      const auto inv = axes[axis].inverse(f.position, q, f.velocity);
      CHECK(inv);
      std::cout << f.position << ' ' << f.jacobian << ' ' << f.velocity << ' '
                << inv.position << ' ' << inv.velocity << '\n';
    }
    return 0;
  }
  Axis axis;
  CHECK(axis.forward(0).error == Error::NotConfigured);
  CHECK(axis.inverse(0, 0).error == Error::NotConfigured);
  for (std::size_t i = 0; i < 4; ++i) {
    CHECK(axis.configure(geometry[i], domains[i]) == Error::None);
    CHECK(near(axis.forward(0).position, 0));
    CHECK(axis.forward(domains[i].upper + .01).error == Error::OutsideDomain);
    CHECK(axis.forward(std::numeric_limits<double>::quiet_NaN()).error ==
          Error::NonfiniteInput);
    CHECK(axis.inverse(0, 0, std::numeric_limits<double>::infinity()).error ==
          Error::NonfiniteInput);
    CHECK(!axis.inverse(100, 0));
    for (int k = 0; k <= 200; ++k) {
      const double q =
          domains[i].lower + (domains[i].upper - domains[i].lower) * k / 200;
      const auto f = axis.forward(q, .02);
      CHECK(f);
      const auto inverse = axis.inverse(f.position, q, f.velocity);
      CHECK(inverse && near(inverse.position, q) &&
            near(inverse.velocity, .02));
      const double h = 1e-6;
      if (k > 0 && k < 200) {
        const double difference =
            (axis.forward(q + h).position - axis.forward(q - h).position) /
            (2 * h);
        CHECK(near(difference, f.jacobian, 1e-8));
      }
    }
  }
  auto invalid = geometry[0];
  invalid.branch = 0;
  CHECK(axis.configure(invalid, domains[0]) == Error::InvalidConfiguration);
  CHECK(axis.forward(0).error == Error::NotConfigured);
  invalid = geometry[0];
  invalid.slider_direction[2] = -2;
  CHECK(axis.configure(invalid, domains[0]) == Error::InvalidConfiguration);
  invalid = geometry[0];
  invalid.rod_length = .001;
  CHECK(axis.configure(invalid, domains[0]) == Error::Unreachable);
  CHECK(axis.configure(geometry[0], {1, -1}) == Error::InvalidConfiguration);
  CHECK(axis.configure(geometry[0], {-4, 4}) == Error::InvalidConfiguration);

  // A synthetic crank has two inverse roots on the same assembly branch.
  Geometry crank;
  crank.body_pin = {0, 1, 0};
  crank.slider_direction = {0, 1, 0};
  crank.rod_length = 2;
  CHECK(axis.configure(crank, {-2, 2}) == Error::None);
  const auto target = axis.forward(.6);
  CHECK(target);
  CHECK(near(axis.inverse(target.position, .7).position, .6));
  CHECK(near(axis.inverse(target.position, -.7).position, -.6));
  CHECK(axis.forward(0).error == Error::Singular); // zero transmission Jacobian
  auto other = crank;
  other.branch = -1;
  Axis opposite;
  CHECK(opposite.configure(other, {-2, 2}) == Error::None);
  CHECK(axis.inverse(-4.2, .7).error == Error::WrongBranch);
  CHECK(!near(opposite.forward(.6).position, target.position));
  CHECK(axis.inverse(target.position, 3).error == Error::OutsideDomain);
  CHECK(axis.configure(crank, {.1, .3}) == Error::None);
  CHECK(axis.inverse(target.position, .2).error == Error::OutsideDomain);
  CHECK(axis.configure(crank, {5.8, 6.8}) == Error::None);
  const auto periodic = axis.forward(6.4);
  CHECK(periodic && near(axis.inverse(periodic.position, 6.4).position, 6.4));
  crank.rod_length = 1;
  CHECK(axis.configure(crank, {-2, 2}) == Error::None);
  CHECK(axis.forward(1.5707963267948966).error == Error::Singular);

  LateralCoupling lateral;
  SlideState slides;
  MlState ml;
  CHECK(lateral.forward(0, 0, slides) == Error::NotConfigured);
  CHECK(lateral.configure({1.348575, .6215}, {-.04, .04}) == Error::None);
  for (int i = -100; i <= 100; ++i) {
    const double angle = i * .0003;
    CHECK(lateral.forward(angle, .01, slides) == Error::None);
    CHECK(near(slides.position[0], 1.348575 * std::tan(angle)));
    CHECK(near(slides.velocity[1],
               .6215 * (1 + std::tan(angle) * std::tan(angle)) * .01));
    CHECK(lateral.inverse(slides, ml) == Error::None);
    CHECK(near(ml.position, angle) && near(ml.velocity, .01));
    CHECK(near(ml.position_residual, 0) && near(ml.velocity_residual, 0));
  }
  slides.position = {.01, -.01};
  CHECK(lateral.inverse(slides, ml) == Error::None);
  CHECK(std::abs(ml.position_residual) > .01);
  const double fitted_t =
      (1.348575 * .01 + .6215 * (-.01)) / (1.348575 * 1.348575 + .6215 * .6215);
  CHECK(near(ml.position, std::atan(fitted_t)));
  slides.position = {1.348575 * std::tan(.08), .6215 * std::tan(.08)};
  CHECK(lateral.inverse(slides, ml) == Error::None && near(ml.position, .08));
  CHECK(lateral.forward(.1, 0, slides) == Error::OutsideDomain);
  CHECK(lateral.configure({0, 1}, {-.04, .04}) == Error::InvalidConfiguration);
  CHECK(lateral.forward(0, 0, slides) == Error::NotConfigured);
  CHECK(lateral.configure({1.348575, .6215}, {-.04, .04}) == Error::None);

  Mapping mapping;
  Coordinates q{}, qdot{}, s{}, sdot{}, back{}, velocity{};
  CHECK(mapping.configure(geometry, domains) == Error::None);
  q = {.03, .01, -.02, -.04};
  qdot = {.01, .02, .03, .04};
  const auto count = allocations.load();
  for (int i = 0; i < 10000; ++i) {
    CHECK(lateral.forward(.01, .02, slides) == Error::None);
    CHECK(lateral.inverse(slides, ml) == Error::None);
    CHECK(mapping.forward(q, qdot, s, sdot) == Error::None);
    CHECK(mapping.inverse(s, sdot, q, back, velocity) == Error::None);
    for (std::size_t j = 0; j < 4; ++j)
      CHECK(near(q[j], back[j]) && near(qdot[j], velocity[j]));
  }
  CHECK(allocations.load() == count);
  const Coordinates sentinel{9, 9, 9, 9};
  s = sdot = sentinel;
  q[3] = 1;
  CHECK(mapping.forward(q, qdot, s, sdot) == Error::OutsideDomain);
  CHECK(s == sentinel && sdot == sentinel); // no partial outputs
  CHECK(allocations.load() == count);
  std::cout << "mapping tests passed\n";
}
