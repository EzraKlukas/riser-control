#pragma once

#include "inverted_pendulum_logic.hpp"
#include <array>
#include <atomic>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <string>

namespace riser_inverted_pendulum_controller {

inline constexpr std::size_t kNumWrenchInterfaces = 5;
inline constexpr const char *kCommandInterface = "velocity";
inline constexpr std::array<const char *, kNumWrenchInterfaces>

    WrenchInterfaceNames{"torque.x", "torque.y", "sample_time", "valid",
                         "subject_mass_kg"};
inline constexpr std::array<const char *, 3> JointInterfaceNames{
    "position", "velocity", "effort"}; // expose time too?

enum WrenchInterfaceIndices { TX, TY, SAMPLE_TIME, VALID, MASS };
enum JointInterfaceIndices { POSITION, VELOCITY, EFFORT };

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
    std::array<hardware_interface::LoanedStateInterface *, 3> state_handles{};
    hardware_interface::LoanedCommandInterface *command_handle = nullptr;
  };
  struct WrenchInterfaces {
    std::array<hardware_interface::LoanedStateInterface *, kNumWrenchInterfaces>
        handles{};
  };

  bool read_joints(std::array<logic::JointSample, kNumAngularJoints> &) const;
  bool read_torque(logic::TorqueSample &) const;
  bool read_inputs(logic::Inputs &) const;
  bool send_commands(const std::array<double, kNumAngularJoints> &);

  const std::array<std::string, kNumAngularJoints> joint_names_ = {
      "backboard_pitch", "ml_angle", "foot_plate_pitch"};
  std::string wrench_source_name_ = "ankle_wrench";
  std::array<JointInterfaces, kNumAngularJoints> joint_interfaces_;
  WrenchInterfaces wrench_interface_;

  void reset();

  logic::InvertedPendulumLogic logic_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_guard_;
  std::atomic<bool> active_{false};
};
} // namespace riser_inverted_pendulum_controller
