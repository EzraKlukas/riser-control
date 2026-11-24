#include <rclcpp/rclcpp.hpp>
#include <riser_calibration/calibration_controller.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  using Callback = controller_interface::CallbackReturn;
  using Result = controller_interface::return_type;
  riser_calibration_controller::CalibrationController controller;
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "bottom_calibration_test";
  params.controller_manager_update_rate = 1000;
  params.robot_description = R"(<robot name="calibration">
    <link name="base"/>
    <ros2_control name="test_hardware" type="system"><hardware>
    <plugin>mock_components/GenericSystem</plugin></hardware>
    <gpio name="bottom_actuator">
      <command_interface name="position"/><state_interface name="position"/>
      <state_interface name="limit_switch_lower"/><state_interface name="limit_switch_upper"/>
    </gpio></ros2_control></robot>)";
  if (controller.init(params) != Result::OK)
    return 1;
  const auto node = controller.get_node();
  node->set_parameters({
      rclcpp::Parameter("lower_limits", std::vector<double>(4, -1.)),
      rclcpp::Parameter("upper_limits", std::vector<double>(4, 1.)),
  });
  const rclcpp_lifecycle::State state;
  if (controller.on_configure(state) != Callback::SUCCESS ||
      controller.command_interface_configuration().names !=
          std::vector<std::string>{"bottom_actuator/position"} ||
      controller.state_interface_configuration().names.size() != 3)
    return 2;

  double target = 0, position = -.8, lower = 1, upper = 0;
  auto command = std::make_shared<hardware_interface::CommandInterface>(
      "bottom_actuator", "position", &target);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(command, nullptr);
  std::vector<hardware_interface::LoanedStateInterface> states;
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> storage;
  for (const auto &[name, value] :
       std::array<std::pair<const char *, double *>, 3>{
           {{"position", &position},
            {"limit_switch_lower", &lower},
            {"limit_switch_upper", &upper}}}) {
    storage.push_back(std::make_shared<hardware_interface::StateInterface>(
        "bottom_actuator", name, value));
    states.emplace_back(storage.back());
  }
  controller.assign_interfaces(std::move(commands), std::move(states));
  if (controller.on_activate(state) != Callback::SUCCESS)
    return 3;
  const auto step = [&] {
    return controller.update(rclcpp::Time(0),
                             rclcpp::Duration::from_seconds(.1));
  };
  if (step() != Result::OK)
    return 4;
  position = .8;
  lower = 0;
  upper = 1;
  if (step() != Result::OK)
    return 5;
  upper = 0;
  for (int i = 0; i < 120; ++i)
    if (step() != Result::OK)
      return 6;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  const std::string prefix = "result.bottom_actuator.";
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline &&
         !node->get_parameter(prefix + "valid").as_bool())
    executor.spin_once(std::chrono::milliseconds(100));
  if (!node->get_parameter(prefix + "valid").as_bool() ||
      std::abs(node->get_parameter(prefix + "negative_travel_m").as_double() -
               .4) > 1e-9 ||
      std::abs(node->get_parameter(prefix + "positive_travel_m").as_double() -
               1.2) > 1e-9 ||
      node->get_parameter("result.top_actuator.valid").as_bool())
    return 7;
  controller.release_interfaces();
  rclcpp::shutdown();
  return 0;
}
