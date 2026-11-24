#include <rclcpp/rclcpp.hpp>
#include <riser_inverted_pendulum/inverted_pendulum_controller.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  using namespace riser_inverted_pendulum_controller;
  using Callback = controller_interface::CallbackReturn;
  using Result = controller_interface::return_type;
  InvertedPendulumController controller;
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "balance_mode_test";
  params.controller_manager_update_rate = 1000;
  params.robot_description = R"(<robot name="balance_test">
    <link name="base"/><link name="board"/>
    <joint name="backboard_pitch" type="continuous"><parent link="base"/><child link="board"/></joint>
    <ros2_control name="balance" type="system"><hardware><plugin>mock_components/GenericSystem</plugin></hardware>
    <joint name="backboard_pitch"><param name="velocity_limit">0.1</param>
    <command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/>
    </joint></ros2_control></robot>)";
  if (controller.init(params) != Result::OK)
    return 1;
  const rclcpp_lifecycle::State lifecycle;
  for (const auto *mode : {"ml", "footplate", "backboard"}) {
    controller.get_node()->set_parameter(rclcpp::Parameter("mode", mode));
    const auto expected =
        std::string(mode) == "backboard" ? Callback::SUCCESS : Callback::ERROR;
    if (controller.on_configure(lifecycle) != expected)
      return 2;
  }
  const auto needed = controller.state_interface_configuration().names;
  const std::vector<std::string> commands_needed{"backboard_pitch/velocity"};
  if (controller.command_interface_configuration().names != commands_needed ||
      needed.size() != 6)
    return 3;
  double rate = NAN;
  auto command = std::make_shared<hardware_interface::CommandInterface>(
      "backboard_pitch", "velocity", &rate);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(command, nullptr);
  std::array<double, 6> values{0.01, 0.0, 2.0, 1.0, 1.0, 70.0};
  const std::array<std::string, 6> names{
      "backboard_pitch/position", "backboard_pitch/velocity",
      "ankle_wrench/torque.x",    "ankle_wrench/sample_time",
      "ankle_wrench/valid",       "ankle_wrench/subject_mass_kg"};
  std::vector<hardware_interface::LoanedStateInterface> states;
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> storage;
  for (size_t i = 0; i < names.size(); ++i) {
    const auto slash = names[i].find('/');
    storage.push_back(std::make_shared<hardware_interface::StateInterface>(
        names[i].substr(0, slash), names[i].substr(slash + 1), &values[i]));
    states.emplace_back(storage.back());
  }
  controller.assign_interfaces(std::move(commands), std::move(states));
  if (controller.on_activate(lifecycle) != Callback::SUCCESS ||
      controller.update(rclcpp::Time(1000000000),
                        rclcpp::Duration::from_seconds(.001)) != Result::OK ||
      !std::isfinite(rate) || rate <= 0)
    return 4;
  if (controller.get_node()
          ->set_parameter(rclcpp::Parameter("mode", "ml"))
          .successful)
    return 5;
  controller.on_deactivate(lifecycle);
  controller.release_interfaces();
  rclcpp::shutdown();
  return 0;
}
