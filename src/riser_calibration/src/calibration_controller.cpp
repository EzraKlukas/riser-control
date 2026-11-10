#include "riser_calibration/calibration_controller.hpp"
#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace riser_calibration_controller {
using Callback = controller_interface::CallbackReturn;
using Result = controller_interface::return_type;

// Resolve names once at activation; update() uses readable, checked handles.
template <class Interfaces>
auto find_interface(Interfaces &interfaces, const std::string &name) {
  const auto it =
      std::find_if(interfaces.begin(), interfaces.end(),
                   [&](const auto &i) { return i.get_name() == name; });
  return it == interfaces.end() ? nullptr : &*it;
}

controller_interface::InterfaceConfiguration
CalibrationController::command_interface_configuration() const {
  std::vector<std::string> names;
  for (const auto &joint : joint_names_)
    names.push_back(joint + "/position");
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          names};
}

controller_interface::InterfaceConfiguration
CalibrationController::state_interface_configuration() const {
  std::vector<std::string> names;
  for (const auto &joint : joint_names_)
    for (const auto *type :
         {"position", "limit_switch_lower", "limit_switch_upper"})
      names.push_back(joint + "/" + type);
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          names};
}

Callback CalibrationController::on_init() {
  try {
    auto_declare<std::vector<double>>("lower_limits", {});
    auto_declare<std::vector<double>>("upper_limits", {});
    auto_declare<std::vector<double>>("speeds", {0.01, 0.01, 0.01, 0.005});
    return Callback::SUCCESS;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return Callback::ERROR;
  }
}

Callback CalibrationController::on_configure(const rclcpp_lifecycle::State &) {
  const auto lower =
      get_node()->get_parameter("lower_limits").as_double_array();
  const auto upper =
      get_node()->get_parameter("upper_limits").as_double_array();
  const auto speeds = get_node()->get_parameter("speeds").as_double_array();
  if (lower.size() != kNumActuators || upper.size() != kNumActuators ||
      speeds.size() != kNumActuators) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected four lower_limits, upper_limits and speeds");
    return Callback::ERROR;
  }
  for (std::size_t i = 0; i < kNumActuators; ++i) {
    if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) ||
        !std::isfinite(speeds[i]) || lower[i] >= upper[i] || speeds[i] <= 0)
      return Callback::ERROR;
    actuators_[i].lower = lower[i];
    actuators_[i].upper = upper[i];
    actuators_[i].speed = speeds[i];
  }
  return Callback::SUCCESS;
}

Callback CalibrationController::on_activate(const rclcpp_lifecycle::State &) {
  for (std::size_t i = 0; i < kNumActuators; ++i) {
    auto &a = actuators_[i];
    const auto &name = joint_names_[i];
    a.position = find_interface(state_interfaces_, name + "/position");
    a.lower_switch =
        find_interface(state_interfaces_, name + "/limit_switch_lower");
    a.upper_switch =
        find_interface(state_interfaces_, name + "/limit_switch_upper");
    a.command = find_interface(command_interfaces_, name + "/position");
    if (!a.position || !a.lower_switch || !a.upper_switch || !a.command) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing interface for %s",
                   name.c_str());
      return Callback::ERROR;
    }
    if (!std::isfinite(a.position->get_value()))
      return Callback::ERROR;
    a.target = std::clamp(a.position->get_value(), a.lower, a.upper);
    a.state = FINDING_NEG_LIMIT;
  }
  for (auto &a : actuators_)
    if (!a.command->set_value(a.target)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not write calibration hold target");
      return Callback::ERROR;
    }
  current_actuator_ = 0;
  phase_seconds_ = 0;
  failed_ = false;
  return Callback::SUCCESS;
}

Callback CalibrationController::on_deactivate(const rclcpp_lifecycle::State &) {
  // Position commands hold the last valid targets; zero would request motion.
  for (auto &a : actuators_)
    if (!a.command->set_value(a.target)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not write calibration hold target");
      return Callback::ERROR;
    }
  return Callback::SUCCESS;
}

Result CalibrationController::update(const rclcpp::Time &,
                                     const rclcpp::Duration &period) {
  // Hold the last targets after an error until reactivated.
  if (failed_)
    return Result::OK;
  const auto fail = [&](const char *reason) {
    failed_ = true;
    RCLCPP_ERROR(get_node()->get_logger(), "Calibration stopped: %s", reason);
    return Result::ERROR; // Leave all position targets at their last valid
                          // values.
  };
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt < 0)
    return fail("invalid update period");
  for (const auto &a : actuators_) {
    const double lower = a.lower_switch->get_value(),
                 upper = a.upper_switch->get_value();
    if (!std::isfinite(a.position->get_value()) || (lower != 0 && lower != 1) ||
        (upper != 0 && upper != 1) || (lower && upper))
      return fail("invalid position or limit-switch state");
  }
  if (dt == 0 || current_actuator_ == kNumActuators)
    return Result::OK;
  if ((phase_seconds_ += dt) > 180.)
    return fail("phase exceeded 180 seconds without reaching its target");

  auto &a = actuators_[current_actuator_];
  const double step = a.speed * dt;
  switch (a.state) {
  case FINDING_NEG_LIMIT:
    if (a.lower_switch->get_value()) {
      a.lower_position = a.position->get_value();
      a.state = FINDING_POS_LIMIT;
      phase_seconds_ = 0;
      RCLCPP_INFO(get_node()->get_logger(), "%s: lower limit %.6f m",
                  joint_names_[current_actuator_].c_str(), a.lower_position);
    } else {
      a.target = std::max(a.lower, a.target - step);
    }
    break;
  case FINDING_POS_LIMIT:
    if (a.upper_switch->get_value()) {
      a.upper_position = a.position->get_value();
      if (a.upper_position <= a.lower_position)
        return fail("upper limit is not above lower limit");
      // Bottom neutral is one quarter of its stroke; the other axes are
      // centred.
      const double fraction =
          current_actuator_ == kNumActuators - 1 ? 0.25 : 0.5;
      a.home = std::clamp(a.lower_position +
                              fraction * (a.upper_position - a.lower_position),
                          a.lower, a.upper);
      a.state = GOING_HOME;
      phase_seconds_ = 0;
      RCLCPP_INFO(
          get_node()->get_logger(), "%s: upper limit %.6f m; home %.6f m",
          joint_names_[current_actuator_].c_str(), a.upper_position, a.home);
    } else {
      a.target = std::min(a.upper, a.target + step);
    }
    break;
  case GOING_HOME:
    a.target += std::clamp(a.home - a.target, -step, step);
    if (a.target == a.home) {
      a.state = HOME;
      phase_seconds_ = 0;
      RCLCPP_INFO(get_node()->get_logger(), "%s: home commanded",
                  joint_names_[current_actuator_].c_str());
      ++current_actuator_;
    }
    break;
  case HOME:
    break;
  }
  if (!a.command->set_value(a.target))
    return fail("could not write actuator target");
  return Result::OK;
}
} // namespace riser_calibration_controller

PLUGINLIB_EXPORT_CLASS(riser_calibration_controller::CalibrationController,
                       controller_interface::ControllerInterface)
