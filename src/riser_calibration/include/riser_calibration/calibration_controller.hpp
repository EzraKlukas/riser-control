#pragma once

#include <array>
#include <controller_interface/controller_interface.hpp>
#include <string>

namespace riser_calibration_controller {

constexpr std::size_t kNumActuators = 4;
enum CalibrationState {
  FINDING_NEG_LIMIT,
  FINDING_POS_LIMIT,
  GOING_HOME,
  HOME
};

class CalibrationController : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &,
                                           const rclcpp::Duration &) override;

private:
  const std::array<std::string, kNumActuators> joint_names_{
      "top_actuator", "torso_actuator", "hip_actuator", "bottom_actuator"};
  struct Actuator {
    hardware_interface::LoanedStateInterface *position = nullptr;
    hardware_interface::LoanedStateInterface *lower_switch = nullptr;
    hardware_interface::LoanedStateInterface *upper_switch = nullptr;
    hardware_interface::LoanedCommandInterface *command = nullptr;
    CalibrationState state = FINDING_NEG_LIMIT;
    double lower = 0, upper = 0, speed = 0;
    double target = 0, lower_position = 0, upper_position = 0, home = 0;
  };
  std::array<Actuator, kNumActuators> actuators_{};
  std::size_t current_actuator_ = 0;
  double phase_seconds_ = 0;
  bool failed_ = false;
};
} // namespace riser_calibration_controller
