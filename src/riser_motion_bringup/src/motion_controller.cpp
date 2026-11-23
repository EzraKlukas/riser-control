#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <riser_motion_bringup/profile.hpp>

namespace riser_motion_bringup {
class MotionController final
    : public controller_interface::ControllerInterface {
  Settings settings_;
  MotorProfile profile_;
  std::string joint_;
  MotorSample sample() const {
    return {state_interfaces_[0].get_optional(1).value_or(NAN),
            state_interfaces_[1].get_optional(1).value_or(NAN),
            state_interfaces_[2].get_optional(1).value_or(NAN)};
  }

public:
  controller_interface::CallbackReturn on_init() override {
    auto_declare<std::string>("joint", "bottom_motor");
    auto_declare<std::string>("profile", "ramp");
    auto_declare<double>("speed_rad_s", 0.05);
    auto_declare<double>("duration_s", 4);
    auto_declare<double>("ramp_s", 1);
    auto_declare<double>("max_travel_rad", 0.2);
    auto_declare<double>("max_acceleration_rad_s2", 0.1);
    auto_declare<double>("max_period_s", 0.005);
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override {
    joint_ = get_node()->get_parameter("joint").as_string();
    settings_.profile = get_node()->get_parameter("profile").as_string();
    settings_.speed = get_node()->get_parameter("speed_rad_s").as_double();
    settings_.duration = get_node()->get_parameter("duration_s").as_double();
    settings_.ramp = get_node()->get_parameter("ramp_s").as_double();
    settings_.max_travel =
        get_node()->get_parameter("max_travel_rad").as_double();
    settings_.max_acceleration =
        get_node()->get_parameter("max_acceleration_rad_s2").as_double();
    settings_.max_period =
        get_node()->get_parameter("max_period_s").as_double();
    if (joint_.empty() || !valid(settings_)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Invalid profile, acceleration, timing or travel budget");
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {joint_ + "/velocity"}};
  }
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {joint_ + "/position", joint_ + "/velocity", joint_ + "/ready"}};
  }
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override {
    if (command_interfaces_.size() != 1 || state_interfaces_.size() != 3 ||
        command_interfaces_[0].get_name() != joint_ + "/velocity" ||
        state_interfaces_[0].get_name() != joint_ + "/position" ||
        state_interfaces_[1].get_name() != joint_ + "/velocity" ||
        state_interfaces_[2].get_name() != joint_ + "/ready" ||
        !profile_.start(settings_, sample()) ||
        !command_interfaces_[0].set_value(0.0, 1))
      return controller_interface::CallbackReturn::ERROR;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override {
    if (!command_interfaces_.empty() && !command_interfaces_[0].set_value(0.0, 1))
      return controller_interface::CallbackReturn::ERROR;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::return_type
  update(const rclcpp::Time &, const rclcpp::Duration &period) override {
    const auto target = profile_.update(period.seconds(), sample());
    if (!command_interfaces_[0].set_value(target.value_or(0), 1) || !target)
      return controller_interface::return_type::ERROR;
    return controller_interface::return_type::OK;
  }
};
} // namespace riser_motion_bringup
PLUGINLIB_EXPORT_CLASS(riser_motion_bringup::MotionController,
                       controller_interface::ControllerInterface)
