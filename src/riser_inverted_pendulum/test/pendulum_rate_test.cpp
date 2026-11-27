#include "riser_inverted_pendulum/inverted_pendulum_logic.hpp"

int main() {
  using namespace riser_inverted_pendulum_controller;
  logic::InvertedPendulumLogic model;
  constexpr double test_rate_limit = 0.02;
  logic::Parameters parameters;
  parameters.max_rate_rad_s = test_rate_limit;
  if (model.configure(parameters) || !model.activate(0.) || model.activate(70.))
    return 1;

  logic::Inputs inputs{};
  inputs.ankle_wrench = {-30., 0., 1., true};
  const auto first = model.update(inputs, 1., 0.001);
  const auto second = model.update(inputs, 1.002, 0.002);
  const double rate = first.velocity_command;
  if (first.error || second.error || rate <= 0. ||
      std::abs(second.velocity_command - 3. * rate) > 1e-12)
    return 1;

  if (model.update(inputs, 1.1, 0.1).velocity_command != test_rate_limit)
    return 1;
  inputs.ankle_wrench.T_x = 30.;
  if (model.update(inputs, 1.101, 0.001).velocity_command >= test_rate_limit)
    return 1;

  model.deactivate();
  if (model.activate(70.))
    return 1;
  inputs.ankle_wrench.T_x = 0.;
  if (model.update(inputs, 2., 0.001).velocity_command != 0.)
    return 1;
  inputs.angle.position = parameters.upper_pendulum_angle_rad;
  const auto stopped = model.update(inputs, 2.001, 0.001);
  if (!stopped.error || stopped.velocity_command != 0.)
    return 1;
}
