#pragma once

#include "riser_wrench_aggregation/wrench_aggregation_logic.hpp"
#include <array>
#include <atomic>
#include <controller_interface/chainable_controller_interface.hpp>
#include <functional>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

namespace riser_wrench_aggregation {
inline constexpr std::array<const char *, 8> WrenchInterfaceNames{
    "force.x",  "force.y",  "force.z",     "torque.x",
    "torque.y", "torque.z", "sample_time", "valid"};

inline constexpr std::array<const char *, 7> PlateInterfaceNames{
    "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z",
    "valid"};

class WrenchAggregationController
    : public controller_interface::ChainableControllerInterface {
public:
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  std::vector<hardware_interface::StateInterface::SharedPtr>
  on_export_state_interfaces_list() override;
  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_reference_interfaces_list() override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type
  update_reference_from_subscribers(const rclcpp::Time &,
                                    const rclcpp::Duration &) override;
  controller_interface::return_type
  update_and_write_commands(const rclcpp::Time &time,
                            const rclcpp::Duration &period) override;

private:
  struct PlateInterfaces {
    std::array<hardware_interface::LoanedStateInterface *, PlateInterfaceNames.size()> handles{};
  };
  static bool read_sample(const PlateInterfaces &, logic::PlateSample &);
  bool read_inputs(logic::Inputs &) const;
  void publish_sample(const logic::WrenchSample &);
  void reset();
  using CaptureReply = std::function<void(bool, const std::string &)>;
  void request_capture(logic::Capture, CaptureReply);
  void respond_to_capture();

  const std::array<std::string, 2> plate_names_{"left_force_plate",
                                                "right_force_plate"};
  std::array<PlateInterfaces, 2> plate_interfaces_{};
  std::array<double, 8> output_values_{};
  logic::WrenchAggregationLogic logic_;
  std::string read_time_interface_name_;
  hardware_interface::LoanedStateInterface *read_time_interface_ = nullptr;
  hardware_interface::LoanedStateInterface *foot_pitch_interface_ = nullptr;
  std::atomic<bool> active_{false};
  double subject_mass_state_ = 0.;
  // The update thread owns logic_ and exported state. The ROS executor owns the
  // deferred reply; these atomics carry the request and result between them.
  std::atomic<logic::Capture> capture_request_{logic::Capture::None};
  std::atomic<logic::CaptureResult> capture_result_{
      logic::CaptureResult::Pending};
  std::atomic<double> measured_mass_kg_{0.};
  double parameter_mass_kg_ = 0.;
  CaptureReply capture_reply_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr tare_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mass_service_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_guard_;
};
} // namespace riser_wrench_aggregation
