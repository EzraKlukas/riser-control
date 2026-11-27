#include "riser_calibration/calibration_controller.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <hardware_interface/component_parser.hpp>
#include <limits>
#include <pluginlib/class_list_macros.hpp>

namespace riser_calibration_controller {
using Callback = controller_interface::CallbackReturn;
using Result = controller_interface::return_type;

static double
read_state(const hardware_interface::LoanedStateInterface &state) {
  return state.get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());
}

constexpr double kInvalidMidpoint = std::numeric_limits<double>::quiet_NaN();

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
  for (const auto i : enabled_)
    for (const auto *type : {"velocity", "calibration_midpoint", "calibration_half_travel"})
      names.push_back(joint_names_[i] + "/" + type);
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          names};
}

controller_interface::InterfaceConfiguration
CalibrationController::state_interface_configuration() const {
  std::vector<std::string> names;
  for (const auto i : enabled_)
    for (const auto *type : {"velocity", "position", "reverse_limit",
                             "forward_limit", "ready", "feedback_valid"})
      names.push_back(joint_names_[i] + "/" + type);
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          names};
}

Callback CalibrationController::on_init() {
  try {
    auto_declare<double>("max_velocity_m_s", 0.);
    auto_declare<double>("max_travel_m", 0.);
    auto_declare<double>("speed_m_s", 0.);
    for (const auto &name : joint_names_) {
      const auto prefix = "result." + name + ".";
      auto_declare<double>(prefix + "negative_travel_m", 0.);
      auto_declare<double>(prefix + "positive_travel_m", 0.);
      auto_declare<bool>(prefix + "valid", false);
    }
    results_timer_ = get_node()->create_wall_timer(
        std::chrono::milliseconds(100), [this] { publish_results(); });
    return Callback::SUCCESS;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return Callback::ERROR;
  }
}

Callback CalibrationController::on_configure(const rclcpp_lifecycle::State &) {
  enabled_.clear();
  std::vector<hardware_interface::HardwareInfo> hardware;
  try {
    hardware = hardware_interface::parse_control_resources_from_urdf(
        get_robot_description());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
    return Callback::ERROR;
  }
  const auto has = [](const auto &interfaces, const char *name) {
    return std::any_of(
        interfaces.begin(), interfaces.end(),
        [name](const auto &iface) { return iface.name == name; });
  };
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    for (const auto &component : hardware) {
      for (const auto *resources : {&component.joints, &component.gpios}) {
        for (const auto &axis : *resources) {
          if (axis.name != joint_names_[i])
            continue;
          if (!has(axis.command_interfaces, "velocity") ||
              !has(axis.command_interfaces, "calibration_midpoint") ||
              !has(axis.command_interfaces, "calibration_half_travel") ||
              !has(axis.state_interfaces, "velocity") ||
              !has(axis.state_interfaces, "position") ||
              !has(axis.state_interfaces, "forward_limit") ||
              !has(axis.state_interfaces, "reverse_limit") ||
              !has(axis.state_interfaces, "ready") ||
              !has(axis.state_interfaces, "feedback_valid")) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "%s lacks required calibration interfaces",
                         axis.name.c_str());
            return Callback::ERROR;
          }
          enabled_.push_back(i);
        }
      }
    }
  }
  if (enabled_.empty()) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "No linear actuator velocity interfaces in the robot description");
    return Callback::ERROR;
  }
  const double speed_cap =
      get_node()->get_parameter("max_velocity_m_s").as_double();
  speed_m_s_ = get_node()->get_parameter("speed_m_s").as_double();
  max_travel_m_ = get_node()->get_parameter("max_travel_m").as_double();
  if (!std::isfinite(speed_cap) || !std::isfinite(speed_m_s_) ||
      !std::isfinite(max_travel_m_) || speed_m_s_ <= 0 ||
      speed_m_s_ > speed_cap || max_travel_m_ <= 0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Invalid calibration speed or commissioning limits");
    return Callback::ERROR;
  }
  return Callback::SUCCESS;
}

Callback CalibrationController::on_activate(const rclcpp_lifecycle::State &) {
  for (const auto i : enabled_) {
    auto &a = actuators_[i];
    const auto &name = joint_names_[i];
    a.velocity = find_interface(state_interfaces_, name + "/velocity");
    a.position = find_interface(state_interfaces_, name + "/position");
    a.lower_switch = find_interface(state_interfaces_, name + "/reverse_limit");
    a.upper_switch = find_interface(state_interfaces_, name + "/forward_limit");
    a.ready = find_interface(state_interfaces_, name + "/ready");
    a.feedback_valid =
        find_interface(state_interfaces_, name + "/feedback_valid");
    a.command = find_interface(command_interfaces_, name + "/velocity");
    a.midpoint = find_interface(command_interfaces_, name + "/calibration_midpoint");
    a.half_travel = find_interface(command_interfaces_, name + "/calibration_half_travel");
    if (!a.midpoint || !a.half_travel || !a.position || !a.velocity || !a.lower_switch || !a.upper_switch ||
        !a.ready || !a.feedback_valid || !a.command) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing interface for %s",
                   name.c_str());
      return Callback::ERROR;
    }
    if (!std::isfinite(read_state(*a.velocity)))
      return Callback::ERROR;
    a.target_velocity = 0.0; // zero velocity to start.
    a.initial_position = read_state(*a.position);
    a.state = FINDING_NEG_LIMIT;
  }
  for (const auto i : enabled_)
    if (!actuators_[i].command->set_value(0.) ||
        !actuators_[i].midpoint->set_value(kInvalidMidpoint) ||
        !actuators_[i].half_travel->set_value(0.)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Could not write calibration hold velocity target_velocity");
      return Callback::ERROR;
    }
  for (auto &result : results_)
    result.valid.store(false);
  current_actuator_ = 0;
  phase_seconds_ = 0;
  settled_seconds_ = 0;
  started_ = false;
  failed_ = false;
  return Callback::SUCCESS;
}

Callback CalibrationController::on_deactivate(const rclcpp_lifecycle::State &) {
  // End by requesting hold velocity.
  for (const auto i : enabled_)
    if (!actuators_[i].command->set_value(0.)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Could not write calibration hold velocity");
      return Callback::ERROR;
    }
  return Callback::SUCCESS;
}

Result CalibrationController::update(const rclcpp::Time &,
                                     const rclcpp::Duration &period) {
  // Hold the last target_velocitys after an error until reactivated.
  if (failed_)
    return Result::OK;
  const auto fail = [&](const char *reason) {
    failed_ = true;
    for (const auto i : enabled_) {
      auto &axis = actuators_[i];
      results_[i].valid.store(false);
      const bool stopped = axis.command->set_value(0.);
      const bool invalidated = axis.midpoint->set_value(kInvalidMidpoint);
      if (!stopped || !invalidated) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Could not write calibration hold velocity");
      }
    }
    RCLCPP_ERROR(get_node()->get_logger(), "Calibration stopped: %s", reason);
    return Result::ERROR;
  };
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt < 0)
    return fail("invalid update period");
  bool waiting_for_enable = false;
  for (const auto i : enabled_) {
    const auto &a = actuators_[i];
    const double ready = read_state(*a.ready);
    if (read_state(*a.feedback_valid) != 1 ||
        (ready != 2 && (ready != 1 || started_)))
      return fail("drive not ready or feedback invalid");
    waiting_for_enable |= ready == 1;
    const double lower = read_state(*a.lower_switch),
                 upper = read_state(*a.upper_switch);
    if (!std::isfinite(read_state(*a.position)) ||
        !std::isfinite(read_state(*a.velocity)) || (lower != 0 && lower != 1) ||
        (upper != 0 && upper != 1) || (lower && upper))
      return fail("invalid position, velocity, or limit-switch state");
  }
  // since we increase current_actuator_ each axis, current_actuator_ ==
  // enabled_.size() means we've calibrated all the axes.
  if (dt == 0 || waiting_for_enable || current_actuator_ == enabled_.size())
    return Result::OK;
  started_ = true;
  if ((phase_seconds_ += dt) > 180.)
    return fail(
        "phase exceeded 180 seconds without reaching its target_velocity");
  const auto index = enabled_[current_actuator_];
  auto &a = actuators_[index];
  switch (a.state) {
  case FINDING_NEG_LIMIT:
    if (read_state(*a.lower_switch)) {
      a.lower_position = read_state(*a.position);
      a.state = FINDING_POS_LIMIT;
      phase_seconds_ = 0;
      RCLCPP_INFO(get_node()->get_logger(), "%s: lower limit %.6f m",
                  joint_names_[index].c_str(), a.lower_position);
      a.target_velocity = 0.; // should I stay at zero velocity for a bit?
    } else {
      if (std::abs(read_state(*a.position) - a.initial_position) > max_travel_m_)
        return fail("reverse switch not found within travel budget");
      a.target_velocity = -speed_m_s_;
    }
    break;
  case FINDING_POS_LIMIT:
    if (read_state(*a.upper_switch)) {
      a.upper_position = read_state(*a.position);
      if (a.upper_position <= a.lower_position ||
          a.upper_position - a.lower_position > max_travel_m_)
        return fail("invalid inter-switch distance");
      // initially: home is halfway between limits.
      a.home = (a.lower_position + a.upper_position) / 2;
      if (!(a.lower_position < a.home && a.home < a.upper_position))
        return fail("home is outside the measured switch interval");
      a.state = GOING_HOME;
      phase_seconds_ = 0;
      RCLCPP_INFO(get_node()->get_logger(),
                  "%s: upper limit %.6f m; home %.6f m",
                  joint_names_[index].c_str(), a.upper_position, a.home);
      a.target_velocity = 0.; // hold, don't keep driving into the limit.
    } else {
      if (read_state(*a.position) - a.lower_position > max_travel_m_) {
        a.target_velocity = 0.;
        return fail("inter-switch distance greater than stored parameter");
      } else
        a.target_velocity = speed_m_s_;
    }
    break;
  case GOING_HOME:
    if (read_state(*a.position) > a.home) {
      const double remaining = read_state(*a.position) - a.home;
      a.target_velocity = -std::min(speed_m_s_, std::max(speed_m_s_ * 0.1, remaining * 5));
    } else {
      a.target_velocity = 0.;
      a.state = SETTLING_AT_HOME;
      phase_seconds_ = settled_seconds_ = 0;
    }
    break;
  case SETTLING_AT_HOME:
    if (std::abs(read_state(*a.velocity)) > speed_m_s_ * 0.02)
      settled_seconds_ = 0;
    else
      settled_seconds_ += dt;
    if (settled_seconds_ >= 0.15) {
      a.state = HOME;
      phase_seconds_ = 0;
      ++current_actuator_;
      if (current_actuator_ == enabled_.size()) {
        for (const auto i : enabled_) {
          const auto &axis = actuators_[i];
          const double half = axis.upper_position - axis.home;
          if (!axis.half_travel->set_value(half) ||
              !axis.midpoint->set_value(axis.home))
            return fail("could not establish midpoint reference");
        }
        for (const auto i : enabled_) {
          const auto &axis = actuators_[i];
          auto &result = results_[i];
          result.negative.store(axis.home - axis.lower_position);
          result.positive.store(axis.upper_position - axis.home);
          result.valid.store(true);
          RCLCPP_INFO(get_node()->get_logger(), "%s: midpoint established",
                      joint_names_[i].c_str());
        }
      }
      return Result::OK;
    }
    break;
  case HOME:
    break;
  }
  if (!a.command->set_value(a.target_velocity))
    return fail("could not write actuator target_velocity");
  return Result::OK;
}
void CalibrationController::publish_results() {
  const auto node = get_node();
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const auto prefix = "result." + joint_names_[i] + ".";
    const auto &result = results_[i];
    const bool valid = result.valid.load();
    const double negative = valid ? result.negative.load() : 0.;
    const double positive = valid ? result.positive.load() : 0.;
    if (node->get_parameter(prefix + "valid").as_bool() != valid ||
        node->get_parameter(prefix + "negative_travel_m").as_double() !=
            negative ||
        node->get_parameter(prefix + "positive_travel_m").as_double() !=
            positive) {
      node->set_parameters_atomically({
          rclcpp::Parameter(prefix + "negative_travel_m", negative),
          rclcpp::Parameter(prefix + "positive_travel_m", positive),
          rclcpp::Parameter(prefix + "valid", valid),
      });
    }
  }
}
} // namespace riser_calibration_controller

PLUGINLIB_EXPORT_CLASS(riser_calibration_controller::CalibrationController,
                       controller_interface::ControllerInterface)
