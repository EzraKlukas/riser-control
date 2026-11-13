#pragma once

#include "riser_wrench_aggregation/wrench_aggregation_logic.hpp"
#include <array>
#include <atomic>
#include <controller_interface/chainable_controller_interface.hpp>
#include <string>
#include <vector>

namespace riser_wrench_aggregation {
inline constexpr std::array<const char *, 8> WrenchInterfaceNames{
    "force.x",  "force.y",  "force.z",     "torque.x",
    "torque.y", "torque.z", "sample_time", "valid"};

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
    std::array<hardware_interface::LoanedStateInterface *, 8> handles{};
  };
  static bool read_sample(const PlateInterfaces &, logic::WrenchSample &);
  bool read_inputs(logic::Inputs &) const;
  void publish_sample(const logic::WrenchSample &);
  void reset();

  const std::array<std::string, 2> plate_names_{"left_force_plate",
                                                "right_force_plate"};
  std::array<PlateInterfaces, 2> plate_interfaces_{};
  std::array<double, 8> output_values_{};
  logic::WrenchAggregationLogic logic_;
  std::string read_time_interface_name_;
  double last_read_time_s_ = logic::Unavailable;
  hardware_interface::LoanedStateInterface *read_time_interface_ = nullptr;
  std::atomic<bool> active_{false};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_guard_;
};
} // namespace riser_wrench_aggregation
