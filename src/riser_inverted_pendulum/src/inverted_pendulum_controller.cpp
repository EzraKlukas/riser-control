#include "riser_inverted_pendulum/inverted_pendulum_controller.hpp"
#include "riser_inverted_pendulum/inverted_pendulum_logic.hpp"
#include <controller_interface/controller_interface_base.hpp>
#include <hardware_interface/component_parser.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace riser_inverted_pendulum_controller {
using Callback = controller_interface::CallbackReturn;
using Result = controller_interface::return_type;

namespace {
double angle_rate_limit(const std::string &description, const std::string &joint_name) {
  for (const auto &component :
       hardware_interface::parse_control_resources_from_urdf(description)) {
    for (const auto &joint : component.joints) {
      if (joint.name != joint_name)
        continue;
      const auto has = [](const auto &interfaces, const char *name) {
        return std::any_of(interfaces.begin(), interfaces.end(),
                           [name](const auto &iface) { return iface.name == name; });
      };
      if (!has(joint.command_interfaces, "velocity") ||
          !std::all_of(JointInterfaceNames.begin(), JointInterfaceNames.end(),
                       [&](const auto *name) { return has(joint.state_interfaces, name); }))
        throw std::invalid_argument("Missing calibrated angular velocity contract for " + joint_name);
      const double rate = std::stod(joint.parameters.at("velocity_limit"));
      if (std::isfinite(rate) && rate > 0.)
        return rate;
      throw std::invalid_argument(
          "Invalid angular velocity_limit in robot description");
    }
  }
  throw std::invalid_argument(
      "Missing angular resource " + joint_name);
}
} // namespace

controller_interface::InterfaceConfiguration
InvertedPendulumController::state_interface_configuration() const {
  std::vector<std::string> names;
  for (const auto *interface : JointInterfaceNames)
    names.push_back(joint_name_ + "/" + interface);
  for (const auto *name : WrenchInterfaceNames)
    names.push_back(wrench_source_name_ + "/" + name);
  if (!read_time_name_.empty())
    names.push_back(read_time_name_);
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          names};
}

controller_interface::InterfaceConfiguration
InvertedPendulumController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {joint_name_ + "/velocity"}};
}

Callback InvertedPendulumController::on_init() {
  reset();
  try {
    const logic::Parameters defaults;
    auto_declare<std::string>("mode", "footplate");
    auto_declare<std::string>("read_time_interface", "");
    auto_declare("lower_pendulum_angle_rad", defaults.lower_pendulum_angle_rad);
    auto_declare("upper_pendulum_angle_rad", defaults.upper_pendulum_angle_rad);
    auto_declare("viscosity", defaults.viscosity);
    mass_override_kg_.store(auto_declare<double>("mass", 0.));
    auto_declare("height", defaults.height);
    auto_declare("max_rate_rad_s", 0.);
    auto_declare("is_male", defaults.is_male);
    parameter_guard_ = get_node()->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = !active_.load();
          if (!result.successful) {
            result.reason = "Deactivate before editing parameters";
            return result;
          }
          for (const auto &parameter : parameters) {
            if (parameter.get_name() == "mass" &&
                (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
                 !std::isfinite(parameter.as_double()) || parameter.as_double() < 0.)) {
              result.successful = false;
              result.reason = "mass must be a finite nonnegative number in kilograms";
              break;
            }
          }
          return result;
        });
    // Cache only committed changes. Activation can read the current mass
    // without taking the parameter service's lock.
    mass_parameter_callback_ = get_node()->add_post_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          for (const auto &parameter : parameters)
            if (parameter.get_name() == "mass")
              mass_override_kg_.store(parameter.as_double());
        });
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

Callback
InvertedPendulumController::on_configure(const rclcpp_lifecycle::State &) {
  try {
    logic::Parameters parameters;
    const auto node = get_node();
    read_time_name_ = node->get_parameter("read_time_interface").as_string();
    const auto mode = node->get_parameter("mode").as_string();
    if (mode == "footplate") {
      joint_name_ = "foot_plate_pitch";
      // Toes-up pitch is about the negative ankle X axis. Project the
      // measured torque first so the model can subtract the reaction once.
      ankle_torque_projection_ = -1.;
    } else if (mode == "backboard") {
      joint_name_ = "backboard_pitch";
      ankle_torque_projection_ = 1.;
    } else {
      throw std::invalid_argument("mode must select footplate or backboard");
    }
    parameters.lower_pendulum_angle_rad =
        node->get_parameter("lower_pendulum_angle_rad").as_double();
    parameters.upper_pendulum_angle_rad =
        node->get_parameter("upper_pendulum_angle_rad").as_double();
    parameters.viscosity = node->get_parameter("viscosity").as_double();
    const double mass = mass_override_kg_.load();
    if (!std::isfinite(mass) || mass < 0.)
      throw std::invalid_argument("mass must be finite and nonnegative");
    parameters.height = node->get_parameter("height").as_double();
    const double requested_rate =
        node->get_parameter("max_rate_rad_s").as_double();
    if (!std::isfinite(requested_rate) || requested_rate < 0.)
      throw std::invalid_argument(
          "max_rate_rad_s must be finite and nonnegative");
    const double model_rate = angle_rate_limit(get_robot_description(), joint_name_);
    parameters.max_rate_rad_s =
        requested_rate == 0. ? model_rate
                             : std::min(requested_rate, model_rate);
    parameters.is_male = node->get_parameter("is_male").as_bool();
    if (const auto *error = logic_.configure(parameters)) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", error);
      return Callback::ERROR;
    }
    RCLCPP_INFO(node->get_logger(), "%s velocity cap %.6f rad/s", joint_name_.c_str(),
                parameters.max_rate_rad_s);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
    return Callback::ERROR;
  }
  return Callback::SUCCESS;
}

Callback
InvertedPendulumController::on_activate(const rclcpp_lifecycle::State &) {
  reset();
  active_.store(true);
  {
    for (std::size_t component = 0; component < JointInterfaceNames.size();
         ++component) {
      const auto name = joint_name_ + "/" + JointInterfaceNames[component];
      const auto found = std::find_if(
          state_interfaces_.begin(), state_interfaces_.end(),
          [&](const auto &interface) { return interface.get_name() == name; });
      if (found == state_interfaces_.end()) { // means the search failed.
        RCLCPP_ERROR(get_node()->get_logger(), "Missing state interface %s",
                     name.c_str());
        reset();
        return Callback::ERROR;
      }
      angle_.state_handles[component] = &*found;
    }
    const auto command_interface_name = joint_name_ + "/" + kCommandInterface;
    const auto found =
        std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                     [&](const auto &interface) {
                       return interface.get_name() == command_interface_name;
                     });
    if (found == command_interfaces_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing command interface %s",
                   command_interface_name.c_str());
      reset();
      return Callback::ERROR;
    }
    angle_.command_handle = &*found;
  }
  for (std::size_t component = 0; component < WrenchInterfaceNames.size();
       ++component) {
    const auto wrench_interface_name =
        wrench_source_name_ + "/" + WrenchInterfaceNames[component];
    const auto found =
        std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                     [&](const auto &interface) {
                       return interface.get_name() == wrench_interface_name;
                     });
    if (found == state_interfaces_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing wrench interface %s",
                   wrench_interface_name.c_str());
      reset();
      return Callback::ERROR;
    }
    wrench_interface_.handles[component] = &*found;
  }
  if (!read_time_name_.empty()) {
    const auto found = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [&](const auto &interface) { return interface.get_name() == read_time_name_; });
    if (found == state_interfaces_.end()) {
      reset();
      return Callback::ERROR;
    }
    read_time_ = &*found;
    previous_read_time_s_ = read_time_->get_optional(1).value_or(logic::Unavailable);
  }

  if (angle_.state_handles[CALIBRATED]->get_optional(1).value_or(0) != 1) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s is not calibrated", joint_name_.c_str());
    reset();
    return Callback::FAILURE;
  }
  if (!send_command(0.)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not initialize angular velocity command");
    reset();
    return Callback::ERROR;
  }
  double mass = mass_override_kg_.load();
  if (mass == 0.)
    mass = wrench_interface_.handles[MASS]->get_optional(1).value_or(logic::Unavailable);
  if (const auto *error = logic_.activate(mass)) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error);
    reset();
    return Callback::FAILURE;
  }
  return Callback::SUCCESS;
}

Callback
InvertedPendulumController::on_deactivate(const rclcpp_lifecycle::State &) {
  const bool sent = !angle_.command_handle || send_command(0.);
  reset();
  return sent ? Callback::SUCCESS : Callback::ERROR;
}

Callback
InvertedPendulumController::on_cleanup(const rclcpp_lifecycle::State &) {
  reset();
  return Callback::SUCCESS;
}

bool InvertedPendulumController::read_torque(
    logic::TorqueSample &sample) const {
  std::array<double, MASS> values{};
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (!wrench_interface_.handles[i])
      return false;
    const auto value = wrench_interface_.handles[i]->get_optional(1);
    if (!value)
      return false;
    values[i] = *value;
  }
  if (values[VALID] != 0. && values[VALID] != 1.)
    return false;
  sample.T_x = ankle_torque_projection_ * values[TX];
  sample.sample_time_s = values[SAMPLE_TIME];
  sample.valid = values[VALID];
  return true;
}

bool InvertedPendulumController::read_inputs(logic::Inputs &inputs) const {
  const auto position = angle_.state_handles[POSITION]->get_optional(1);
  const auto velocity = angle_.state_handles[VELOCITY]->get_optional(1);
  if (!position || !velocity ||
      angle_.state_handles[CALIBRATED]->get_optional(1).value_or(0) != 1)
    return false;
  inputs.angle.position = *position;
  inputs.angle.velocity = *velocity;
  inputs.reverse_limit = angle_.state_handles[REVERSE]->get_optional(1).value_or(1) != 0;
  inputs.forward_limit = angle_.state_handles[FORWARD]->get_optional(1).value_or(1) != 0;
  return read_torque(inputs.ankle_wrench);
}

bool InvertedPendulumController::send_command(double command) {
  return angle_.command_handle->set_value(command);
}

void InvertedPendulumController::reset() {
  logic_.deactivate();
  angle_ = {};
  wrench_interface_ = {};
  read_time_ = nullptr;
  previous_read_time_s_ = logic::Unavailable;
  active_.store(false);
}

Result InvertedPendulumController::update(const rclcpp::Time &time,
                                          const rclcpp::Duration &period) {
  // translate from state interfaces to well-packed logical structs.
  logic::Inputs inputs;
  if (!read_inputs(inputs)) {
    send_command(0.);
    return Result::ERROR;
  }
  // call logic update function with latest state update.
  double dt = period.seconds();
  if (read_time_) {
    const double acquired = read_time_->get_optional(1).value_or(logic::Unavailable);
    dt = acquired - previous_read_time_s_;
    previous_read_time_s_ = acquired;
  }
  const auto result = logic_.update(inputs, time.seconds(), dt);
  // translate commands back from abstracted logic to ros2 command interfaces.
  if (!send_command(result.velocity_command)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not send velocity command.");
    return Result::ERROR;
  }
  if (result.error)
    return Result::ERROR;
  return Result::OK;
}

} // namespace riser_inverted_pendulum_controller

PLUGINLIB_EXPORT_CLASS(
    riser_inverted_pendulum_controller::InvertedPendulumController,
    controller_interface::ControllerInterface)
