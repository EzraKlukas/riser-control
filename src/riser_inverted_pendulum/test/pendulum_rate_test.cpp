#include "riser_inverted_pendulum/inverted_pendulum_logic.hpp"

int main() {
  using namespace riser_inverted_pendulum_controller;
  logic::InvertedPendulumLogic model;
  if (model.configure({}) || model.activate(70.))
    return 1;

  logic::Inputs inputs{};
  inputs.ankle_wrench = {30., 0., 1., true};
  const auto first = model.update(inputs, 1., 0.001);
  const auto second = model.update(inputs, 1.002, 0.002);
  const double rate = first.velocity_commands[THETA_BB];
  if (first.error || second.error || rate <= 0. ||
      std::abs(second.velocity_commands[THETA_BB] - 3. * rate) > 1e-12)
    return 1;

  if (model.update(inputs, 1.1, 0.1).velocity_commands[THETA_BB] != 0.012)
    return 1;
  inputs.ankle_wrench.T_x = -30.;
  if (model.update(inputs, 1.101, 0.001).velocity_commands[THETA_BB] >= 0.012)
    return 1;

  model.deactivate();
  if (model.activate(70.))
    return 1;
  inputs.ankle_wrench.T_x = 0.;
  if (model.update(inputs, 2., 0.001).velocity_commands[THETA_BB] != 0.)
    return 1;
}
