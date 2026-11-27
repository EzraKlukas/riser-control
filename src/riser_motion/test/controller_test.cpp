#include "../src/motion_controller.cpp"
#include <hardware_interface/handle.hpp>
#include <rclcpp/rclcpp.hpp>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  riser_motion::MotionController controller;
  using Callback = controller_interface::CallbackReturn;
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "profile_test";
  params.controller_manager_update_rate = 1000;
  if (controller.init(params) != controller_interface::return_type::OK)
    return 1;
  const rclcpp_lifecycle::State state;
  if (controller.on_configure(state) != Callback::ERROR)
    return 2;
  controller.get_node()->set_parameters({
      rclcpp::Parameter("speed_m_s", .01),
      rclcpp::Parameter("max_velocity_m_s", .02),
      rclcpp::Parameter("max_travel_m", .16),
      rclcpp::Parameter("max_acceleration_m_s2", .08),
      rclcpp::Parameter("max_duration_s", 10.),
  });
  if (controller.on_configure(state) != Callback::SUCCESS)
    return 2;
  double command = NAN, position = 0, velocity = 0, ready = 1;
  auto cmd = std::make_shared<hardware_interface::CommandInterface>(
      "bottom_motor", "velocity", &command);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(cmd, nullptr);
  const auto position_state = std::make_shared<hardware_interface::StateInterface>(
      "bottom_motor", "position", &position);
  const auto velocity_state = std::make_shared<hardware_interface::StateInterface>(
      "bottom_motor", "velocity", &velocity);
  const auto ready_state = std::make_shared<hardware_interface::StateInterface>(
      "bottom_motor", "ready", &ready);
  std::vector<hardware_interface::LoanedStateInterface> states;
  states.emplace_back(position_state);
  states.emplace_back(velocity_state);
  states.emplace_back(ready_state);
  controller.assign_interfaces(std::move(commands), std::move(states));
  if (controller.on_activate(state) != Callback::SUCCESS || command != 0)
    return 3;
  controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  if (command != 0)
    return 4;
  ready = 2;
  controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  if (command <= 0)
    return 5;
  ready = 0;
  controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  if (command != 0)
    return 6;
  ready = 2;
  controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  if (command != 0)
    return 7;
  controller.on_deactivate(state);
  ready = 0;
  if (controller.on_activate(state) != Callback::ERROR)
    return 9;
  ready = 2;
  position = NAN;
  if (controller.on_activate(state) != Callback::ERROR)
    return 8;
  controller.release_interfaces();
  rclcpp::shutdown();
  return 0;
}
