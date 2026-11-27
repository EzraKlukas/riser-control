#include "riser_gazebo/reference_mapping.hpp"
#include <iostream>

int main() {
  using namespace riser_gazebo;
  static_assert(angles_for_motors(1u << BottomActuator) == (1u << FootPlate));
  static_assert(angles_for_motors(1u << TopActuator) == (1u << Backboard));
  static_assert(angles_for_motors(1u << TorsoActuator) == 0);
  static_assert(angles_for_motors(1u << HipActuator) == 0);
  static_assert(angles_for_motors((1u << TorsoActuator) | (1u << HipActuator)) ==
                (1u << Ml));
  constexpr double period_s = .001;
  constexpr double board_lower = -.23006989930207766;
  constexpr double board_upper = .216348932826856;
  constexpr double board_rate = board_upper - board_lower;
  constexpr double foot_lower = -.4785836006777029;
  constexpr double foot_upper = .22070887757756963;
  constexpr double foot_rate = foot_upper - foot_lower;
  const Limits limits{{{board_lower, board_upper, board_rate},
                       {foot_lower, foot_upper, foot_rate},
                       {-.24348197180382475, .24348197180382475,
                        .013828819756226319 * (.67 / .02)},
                       {-.335, .335, .67},
                       {-.335, .335, .67},
                       {-.335, .335, .67},
                       {-.08, .24, .32}}};
  ReferenceMapping mapping;
  if (mapping.configure(limits, {1.348575, .6215},
                        {.001003136473, -.2688279981, -.2688279981, 0.}) !=
      kin::Error::None)
    return 1;

  for (const auto mode :
       {CommandMode::AngularVelocity, CommandMode::ActuatorVelocity}) {
    kin::Coordinates physical{board_lower, 0., 0., foot_lower}, qdot{},
        jacobian{};
    PublicCoordinates p{}, v{}, command{};
    if (mapping.observe(physical, qdot, p, v, jacobian) != kin::Error::None)
      return 1;
    // Move pitch in the positive direction. The linkage signs are measured
    // from the same configuration used by the reference mapping.
    if (angular(mode)) {
      command[Backboard] = board_rate;
      command[FootPlate] = foot_rate;
    } else {
      command[TopActuator] = std::copysign(.67, jacobian[kin::FramePitch]);
      command[BottomActuator] = std::copysign(.32, jacobian[kin::FootPitch]);
    }
    for (int i = 0; i < 1500; ++i) {
      const auto old = physical;
      if (mapping.update(mode, command, period_s, p, v, physical, qdot) !=
              kin::Error::None ||
          physical[kin::FramePitch] - old[kin::FramePitch] >
              board_rate * period_s + 1e-10 ||
          physical[kin::FootPitch] - old[kin::FootPitch] >
              foot_rate * period_s + 1e-10) {
        std::cerr << "Invalid mapped reference at sample " << i << '\n';
        return 1;
      }
      if (angular(mode) && i == 999 &&
          (std::abs(p[Backboard] - board_upper) > 1e-9 ||
           std::abs(p[FootPlate] - foot_upper) > 1e-9))
        return 1;
    }
    if (std::abs(p[Backboard] - board_upper) > 1e-9 ||
        std::abs(p[FootPlate] - foot_upper) > 1e-9)
      return 1;
  }
}
