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
    <joint name="foot_plate_pitch" type="continuous"><parent link="base"/><child link="board"/></joint>
    <ros2_control name="balance" type="system"><hardware><plugin>mock_components/GenericSystem</plugin></hardware>
    <joint name="foot_plate_pitch"><param name="velocity_limit">0.1</param><state_interface name="valid"/><state_interface name="reverse_limit"/><state_interface name="forward_limit"/>
    <command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/>
    </joint></ros2_control></robot>)";
  if (controller.init(params) != Result::OK)
    return 1;
  const rclcpp_lifecycle::State lifecycle;
  for (const auto *mode : {"ml", "backboard", "footplate"}) {
    controller.get_node()->set_parameter(rclcpp::Parameter("mode", mode));
    const auto expected =
        std::string(mode) == "footplate" ? Callback::SUCCESS : Callback::ERROR;
    if (controller.on_configure(lifecycle) != expected)
      return 2;
  }
  const auto needed = controller.state_interface_configuration().names;
  const std::vector<std::string> commands_needed{"foot_plate_pitch/velocity"};
  if (controller.command_interface_configuration().names != commands_needed ||
      needed.size() != 9)
    return 3;
  double rate = NAN;
  auto command = std::make_shared<hardware_interface::CommandInterface>(
      "foot_plate_pitch", "velocity", &rate);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(command, nullptr);
  // A positive physical ankle torque projects to negative toes-up torque.
  // At zero mechanism angle it must produce a positive mechanism rate.
  std::array<double, 9> values{0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 1.0, 1.0, 0.0};
  const std::array<std::string, 9> names{
      "foot_plate_pitch/position", "foot_plate_pitch/velocity",
      "foot_plate_pitch/valid", "foot_plate_pitch/reverse_limit", "foot_plate_pitch/forward_limit",
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
  if (controller.on_activate(lifecycle) != Callback::FAILURE)
    return 4;
  values[2] = 1;
  if (controller.on_activate(lifecycle) != Callback::FAILURE || rate != 0.)
    return 4;
  const auto set_mass = [&](double mass) {
    return controller.get_node()->set_parameter(rclcpp::Parameter("mass", mass)).successful;
  };
  if (set_mass(-1.) || !set_mass(70.))
    return 4;
  if (controller.on_activate(lifecycle) != Callback::SUCCESS ||
      controller.update(rclcpp::Time(1000000000),
                        rclcpp::Duration::from_seconds(.001)) != Result::OK ||
      !std::isfinite(rate) || rate <= 0)
    return 4;
  if (set_mass(75.) ||
      controller.get_node()->set_parameter(rclcpp::Parameter("mode", "ml")).successful)
    return 5;
  if (controller.on_deactivate(lifecycle) != Callback::SUCCESS || !set_mass(0.))
    return 6;
  values[8] = 70.;
  if (controller.on_activate(lifecycle) != Callback::SUCCESS)
    return 6;
  controller.on_deactivate(lifecycle);
  controller.release_interfaces();
  rclcpp::shutdown();
  return 0;
}
