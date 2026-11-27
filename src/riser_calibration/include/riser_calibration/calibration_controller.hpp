#pragma once

#include <array>
#include <atomic>
#include <controller_interface/controller_interface.hpp>
#include <string>
#include <vector>

namespace riser_calibration_controller {

constexpr std::size_t kNumActuators = 4;
enum CalibrationState {
  FINDING_NEG_LIMIT,
  FINDING_POS_LIMIT,
  GOING_HOME,
  SETTLING_AT_HOME,
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
      "top_motor", "torso_motor", "hip_motor", "bottom_motor"};
  struct Actuator {
    hardware_interface::LoanedStateInterface *velocity = nullptr;
    hardware_interface::LoanedStateInterface *position = nullptr;
    hardware_interface::LoanedStateInterface *lower_switch = nullptr;
    hardware_interface::LoanedStateInterface *upper_switch = nullptr;
    hardware_interface::LoanedStateInterface *ready = nullptr;
    hardware_interface::LoanedStateInterface *feedback_valid = nullptr;
    hardware_interface::LoanedCommandInterface *command = nullptr;
    hardware_interface::LoanedCommandInterface *midpoint = nullptr;
    hardware_interface::LoanedCommandInterface *half_travel = nullptr;
    CalibrationState state = FINDING_NEG_LIMIT;
    double target_velocity = 0, lower_position = 0, upper_position = 0,
           initial_position = 0, home = 0;
  };
  std::array<Actuator, kNumActuators> actuators_{};
  std::vector<std::size_t> enabled_;
  struct ResultParameters {
    std::atomic<double> negative{0}, positive{0};
    std::atomic<bool> valid{false};
  };
  std::array<ResultParameters, kNumActuators> results_{};
  rclcpp::TimerBase::SharedPtr results_timer_;
  void publish_results();
  std::size_t current_actuator_ = 0;
  double speed_m_s_ = 0, max_travel_m_ = 0;
  double phase_seconds_ = 0;
  double settled_seconds_ = 0;
  bool started_ = false;
  bool failed_ = false;
};
} // namespace riser_calibration_controller
