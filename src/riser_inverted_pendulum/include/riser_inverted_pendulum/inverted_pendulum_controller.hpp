#pragma once

#include "inverted_pendulum_logic.hpp"
#include <array>
#include <atomic>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <string>

namespace riser_inverted_pendulum_controller {

inline constexpr std::size_t kNumWrenchInterfaces = 4;
inline constexpr const char *kCommandInterface = "velocity";
inline constexpr std::array<const char *, kNumWrenchInterfaces>

    WrenchInterfaceNames{"torque.x", "sample_time", "valid", "subject_mass_kg"};
inline constexpr std::array<const char *, 5> JointInterfaceNames{
    "position", "velocity", "valid", "reverse_limit", "forward_limit"};

enum WrenchInterfaceIndices { TX, SAMPLE_TIME, VALID, MASS };
enum JointInterfaceIndices { POSITION, VELOCITY, CALIBRATED, REVERSE, FORWARD };

class InvertedPendulumController
    : public controller_interface::ControllerInterface {
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
  controller_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &,
                                           const rclcpp::Duration &) override;

private:
  struct JointInterfaces {
    std::array<hardware_interface::LoanedStateInterface *,
               JointInterfaceNames.size()>
        state_handles{};
    hardware_interface::LoanedCommandInterface *command_handle = nullptr;
  };
  struct WrenchInterfaces {
    std::array<hardware_interface::LoanedStateInterface *, kNumWrenchInterfaces>
        handles{};
  };

  bool read_torque(logic::TorqueSample &) const;
  bool read_inputs(logic::Inputs &) const;
  bool send_command(double);

  std::string joint_name_ = "foot_plate_pitch";
  double ankle_torque_projection_ = -1.;
  std::string wrench_source_name_ = "ankle_wrench";
  std::string read_time_name_;
  hardware_interface::LoanedStateInterface *read_time_ = nullptr;
  double previous_read_time_s_ = logic::Unavailable;
  JointInterfaces angle_;
  WrenchInterfaces wrench_interface_;

  void reset();

  logic::InvertedPendulumLogic logic_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_guard_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
      mass_parameter_callback_;
  std::atomic<double> mass_override_kg_{0.};
  std::atomic<bool> active_{false};
};
} // namespace riser_inverted_pendulum_controller
