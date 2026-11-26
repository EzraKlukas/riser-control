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
    <link name="base"/><link name="carriage"/>
    <joint name="bottom_motor" type="prismatic">
      <parent link="base"/><child link="carriage"/>
      <limit lower="-1" upper="1" effort="0" velocity="0.02"/>
    </joint>
    <ros2_control name="test_hardware" type="system"><hardware>
    <plugin>mock_components/GenericSystem</plugin></hardware>
    <joint name="bottom_motor">
      <command_interface name="velocity"/><state_interface name="position"/>
      <state_interface name="velocity"/><state_interface name="ready"/>
      <state_interface name="feedback_valid"/>
      <state_interface name="reverse_limit"/><state_interface name="forward_limit"/>
    </joint></ros2_control></robot>)";
  if (controller.init(params) != Result::OK)
    return 1;
  const auto node = controller.get_node();
  node->set_parameters({
      rclcpp::Parameter("max_velocity_m_s", .02),
      rclcpp::Parameter("max_travel_m", .16),
      rclcpp::Parameter("speed_m_s", .001),
  });
  const rclcpp_lifecycle::State state;
  if (controller.on_configure(state) != Callback::SUCCESS ||
      controller.command_interface_configuration().names !=
          std::vector<std::string>{"bottom_motor/velocity"} ||
      controller.state_interface_configuration().names.size() != 6)
    return 2;

  double target = NAN, position = 0, velocity = 0, lower = 0, upper = 0;
  double ready = 1, feedback_valid = 1;
  auto command = std::make_shared<hardware_interface::CommandInterface>(
      "bottom_motor", "velocity", &target);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(command, nullptr);
  std::vector<hardware_interface::LoanedStateInterface> states;
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> storage;
  for (const auto &[name, value] :
       std::array<std::pair<const char *, double *>, 6>{
           {{"position", &position},
            {"velocity", &velocity},
            {"reverse_limit", &lower},
            {"forward_limit", &upper},
            {"ready", &ready},
            {"feedback_valid", &feedback_valid}}}) {
    storage.push_back(std::make_shared<hardware_interface::StateInterface>(
        "bottom_motor", name, value));
    states.emplace_back(storage.back());
  }
  controller.assign_interfaces(std::move(commands), std::move(states));
  if (controller.on_activate(state) != Callback::SUCCESS)
    return 3;
  const auto step = [&] {
    return controller.update(rclcpp::Time(0),
                             rclcpp::Duration::from_seconds(.1));
  };
  if (step() != Result::OK || target != 0)
    return 4;
  ready = 2;
  if (step() != Result::OK || target != -.001)
    return 4;
  position = -.02;
  lower = 1;
  if (step() != Result::OK || target != 0 ||
      step() != Result::OK || target != .001)
    return 5;
  position = .04;
  lower = 0;
  upper = 1;
  if (step() != Result::OK || target != 0 ||
      step() != Result::OK || target != -.001)
    return 5;
  upper = 0;
  position = .009;
  if (step() != Result::OK || target != 0)
    return 6;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  const std::string prefix = "result.bottom_motor.";
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline &&
         !node->get_parameter(prefix + "valid").as_bool())
    executor.spin_once(std::chrono::milliseconds(100));
  if (!node->get_parameter(prefix + "valid").as_bool() ||
      std::abs(node->get_parameter(prefix + "negative_travel_m").as_double() -
               .03) > 1e-9 ||
      std::abs(node->get_parameter(prefix + "positive_travel_m").as_double() -
               .03) > 1e-9 ||
      node->get_parameter("result.top_motor.valid").as_bool())
    return 7;
  for (auto *invalid : {&ready, &feedback_valid}) {
    if (controller.on_activate(state) != Callback::SUCCESS ||
        step() != Result::OK || target != -.001)
      return 8;
    *invalid = (invalid == &ready) ? 1 : 0;
    if (step() != Result::ERROR || target != 0)
      return 9;
    ready = 2;
  }
  controller.release_interfaces();
  rclcpp::shutdown();
  return 0;
}
