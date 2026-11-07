#include "hardware_interface/system_interface.hpp"
#include "riser_hardware/actuator_mapping.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <limits>
#include <vector>

namespace riser_hardware {
class EthercatBoundary final : public hardware_interface::SystemInterface {
  std::vector<double> position_, velocity_, effort_, command_;

public:
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override {
    if (SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;
    if (info_.joints.size() != 3)
      return hardware_interface::CallbackReturn::ERROR;
    for (const auto &joint : info_.joints) {
      if (joint.command_interfaces.size() != 1 ||
          (joint.command_interfaces[0].name != "position" &&
           joint.command_interfaces[0].name != "velocity"))
        return hardware_interface::CallbackReturn::ERROR;
      if (joint.state_interfaces.size() != 3 ||
          joint.state_interfaces[0].name !=
              hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[1].name !=
              hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto n = info_.joints.size();
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    position_.assign(n, unavailable);
    velocity_.assign(n, unavailable);
    effort_.assign(n, unavailable);
    command_.assign(n, unavailable);
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> out;
    out.reserve(3 * info_.joints.size());
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
                       &position_[i]);
      out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
                       &velocity_[i]);
      out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
                       &effort_[i]);
    }
    return out;
  }
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> out;
    out.reserve(info_.joints.size());
    for (std::size_t i = 0; i < info_.joints.size(); ++i)
      out.emplace_back(info_.joints[i].name,
                       info_.joints[i].command_interfaces[0].name,
                       &command_[i]);
    return out;
  }
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override {
    RCLCPP_ERROR(rclcpp::get_logger("riser_ethercat"),
                 "RISER EtherCAT is NOT IMPLEMENTED. Supply verified topology, "
                 "PDOs, homing, "
                 "drive calibration and actuator/joint mapping. Configuration "
                 "refused; no bus opened.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override {
    return hardware_interface::CallbackReturn::ERROR;
  }
  hardware_interface::return_type read(const rclcpp::Time &,
                                       const rclcpp::Duration &) override {
    return hardware_interface::return_type::ERROR;
  }
  hardware_interface::return_type write(const rclcpp::Time &,
                                        const rclcpp::Duration &) override {
    return hardware_interface::return_type::ERROR;
  }
};
} // namespace riser_hardware
PLUGINLIB_EXPORT_CLASS(riser_hardware::EthercatBoundary,
                       hardware_interface::SystemInterface)
