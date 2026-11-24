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
double backboard_rate_limit(const std::string &description) {
  for (const auto &component :
       hardware_interface::parse_control_resources_from_urdf(description)) {
    for (const auto &joint : component.joints) {
      if (joint.name != "backboard_pitch")
        continue;
      const double rate = std::stod(joint.parameters.at("velocity_limit"));
      if (std::isfinite(rate) && rate > 0.)
        return rate;
      throw std::invalid_argument(
          "Invalid backboard velocity_limit in robot description");
    }
  }
  throw std::invalid_argument(
      "Missing backboard_pitch resource in robot description");
}
} // namespace

controller_interface::InterfaceConfiguration
InvertedPendulumController::state_interface_configuration() const {
  std::vector<std::string> names;
  for (const auto *interface : JointInterfaceNames)
    names.push_back(joint_name_ + "/" + interface);
  for (const auto *name : WrenchInterfaceNames)
    names.push_back(wrench_source_name_ + "/" + name);
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
    auto_declare<std::string>("mode", "backboard");
    auto_declare("viscosity", defaults.viscosity);
    auto_declare("mass", defaults.mass);
    auto_declare("height", defaults.height);
    auto_declare("max_backboard_rate_rad_s", 0.);
    auto_declare("is_male", defaults.is_male);
    parameter_guard_ = get_node()->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = !active_.load();
          result.reason =
              result.successful
                  ? ""
                  : "Deactivate and reconfigure before editing parameters";
          return result;
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
    if (node->get_parameter("mode").as_string() != "backboard")
      throw std::invalid_argument("Only backboard has an implemented control "
                                  "law; footplate and ML are unavailable");
    parameters.viscosity = node->get_parameter("viscosity").as_double();
    parameters.mass = node->get_parameter("mass").as_double();
    parameters.height = node->get_parameter("height").as_double();
    const double requested_rate =
        node->get_parameter("max_backboard_rate_rad_s").as_double();
    if (!std::isfinite(requested_rate) || requested_rate < 0.)
      throw std::invalid_argument(
          "max_backboard_rate_rad_s must be finite and nonnegative");
    const double model_rate = backboard_rate_limit(get_robot_description());
    parameters.max_backboard_rate_rad_s =
        requested_rate == 0. ? model_rate
                             : std::min(requested_rate, model_rate);
    parameters.is_male = node->get_parameter("is_male").as_bool();
    if (const auto *error = logic_.configure(parameters)) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", error);
      return Callback::ERROR;
    }
    RCLCPP_INFO(node->get_logger(), "Backboard velocity cap %.6f rad/s",
                parameters.max_backboard_rate_rad_s);
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
      backboard_.state_handles[component] = &*found;
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
    backboard_.command_handle = &*found;
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

  const double measured_mass_kg =
      wrench_interface_.handles[MASS]->get_optional(1).value_or(
          logic::Unavailable);
  if (const auto *error = logic_.activate(measured_mass_kg)) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error);
    reset();
    return Callback::ERROR;
  }
  return Callback::SUCCESS;
}

Callback
InvertedPendulumController::on_deactivate(const rclcpp_lifecycle::State &) {
  reset();
  return Callback::SUCCESS;
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
  sample.T_x = values[TX];
  sample.sample_time_s = values[SAMPLE_TIME];
  sample.valid = values[VALID];
  return true;
}

bool InvertedPendulumController::read_inputs(logic::Inputs &inputs) const {
  const auto position = backboard_.state_handles[POSITION]->get_optional(1);
  const auto velocity = backboard_.state_handles[VELOCITY]->get_optional(1);
  if (!position || !velocity)
    return false;
  inputs.theta_bb.position = *position;
  inputs.theta_bb.velocity = *velocity;
  return read_torque(inputs.ankle_wrench);
}

bool InvertedPendulumController::send_commands(
    const std::array<double, kNumAngularJoints> &commands) {
  return backboard_.command_handle->set_value(commands[THETA_BB]);
}

void InvertedPendulumController::reset() {
  logic_.deactivate();
  backboard_ = {};
  wrench_interface_ = {};
  active_.store(false);
}

Result InvertedPendulumController::update(const rclcpp::Time &time,
                                          const rclcpp::Duration &period) {
  // translate from state interfaces to well-packed logical structs.
  logic::Inputs inputs;
  if (!read_inputs(inputs)) {
    send_commands({});
    return Result::ERROR;
  }
  // call logic update function with latest state update.
  const auto result = logic_.update(inputs, time.seconds(), period.seconds());
  // translate commands back from abstracted logic to ros2 command interfaces.
  if (!send_commands(result.velocity_commands)) {
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
