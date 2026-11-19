#include "riser_inverted_pendulum/inverted_pendulum_controller.hpp"
#include "riser_inverted_pendulum/inverted_pendulum_logic.hpp"
#include <controller_interface/controller_interface_base.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace riser_inverted_pendulum_controller {
using Callback = controller_interface::CallbackReturn;
using Result = controller_interface::return_type;

controller_interface::InterfaceConfiguration
InvertedPendulumController::state_interface_configuration() const {
  std::vector<std::string> names;
  for (const auto &joint : joint_names_) {
    for (const auto *interface : JointInterfaceNames) {
      names.push_back(joint + "/" + interface);
    }
  }
  for (const auto *name : WrenchInterfaceNames)
    names.push_back(wrench_source_name_ + "/" + name);
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          names};
}

controller_interface::InterfaceConfiguration
InvertedPendulumController::command_interface_configuration() const {
  std::vector<std::string> names;
  for (const auto &joint : joint_names_) {
    names.push_back(joint + "/velocity");
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          names};
}

Callback InvertedPendulumController::on_init() {
  reset();
  try {
    const logic::Parameters defaults;
    auto_declare("viscosity", defaults.viscosity);
    auto_declare("mass", defaults.mass);
    auto_declare("height", defaults.height);
    auto_declare("max_backboard_rate_rad_s", defaults.max_backboard_rate_rad_s);
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
    parameters.viscosity = node->get_parameter("viscosity").as_double();
    parameters.mass = node->get_parameter("mass").as_double();
    parameters.height = node->get_parameter("height").as_double();
    parameters.max_backboard_rate_rad_s =
        node->get_parameter("max_backboard_rate_rad_s").as_double();
    parameters.is_male = node->get_parameter("is_male").as_bool();
    if (const auto *error = logic_.configure(parameters)) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", error);
      return Callback::ERROR;
    }
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
  for (std::size_t joint = 0; joint < kNumAngularJoints; ++joint) {
    for (std::size_t component = 0; component < JointInterfaceNames.size();
         ++component) {
      const auto name =
          joint_names_[joint] + "/" + JointInterfaceNames[component];
      const auto found = std::find_if(
          state_interfaces_.begin(), state_interfaces_.end(),
          [&](const auto &interface) { return interface.get_name() == name; });
      if (found == state_interfaces_.end()) { // means the search failed.
        RCLCPP_ERROR(get_node()->get_logger(), "Missing state interface %s",
                     name.c_str());
        reset();
        return Callback::ERROR;
      }
      joint_interfaces_[joint].state_handles[component] = &*found;
    }
    const auto command_interface_name =
        joint_names_[joint] + "/" + kCommandInterface;
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
    joint_interfaces_[joint].command_handle = &*found;
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

  const double measured_mass_kg = wrench_interface_.handles[MASS]
                                      ->get_optional(1)
                                      .value_or(logic::Unavailable);
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
  sample.T_y = values[TY];
  sample.sample_time_s = values[SAMPLE_TIME];
  sample.valid = values[VALID];
  return true;
}

bool InvertedPendulumController::read_joints(
    std::array<logic::JointSample, kNumAngularJoints> &sample) const {
  for (std::size_t joint = 0; joint < kNumAngularJoints; ++joint) {
    std::array<double, JointInterfaceNames.size()> values{};
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (!joint_interfaces_[joint].state_handles[i])
        return false;
      const auto value =
          joint_interfaces_[joint].state_handles[i]->get_optional(1);
      if (!value)
        return false;
      values[i] = *value;
      sample[joint].position = values[POSITION];
      sample[joint].velocity = values[VELOCITY];
      sample[joint].effort = values[EFFORT];
    }
  }
  return true;
}

bool InvertedPendulumController::read_inputs(logic::Inputs &inputs) const {
  std::array<logic::JointSample, kNumAngularJoints> joints{};
  if (!read_joints(joints) || !read_torque(inputs.ankle_wrench))
    return false;
  inputs.theta_bb = joints[THETA_BB];
  inputs.theta_ml = joints[THETA_ML];
  inputs.theta_fp = joints[THETA_FP];
  return true;
}

bool InvertedPendulumController::send_commands(
    const std::array<double, kNumAngularJoints> &commands) {
  for (std::size_t joint = 0; joint < kNumAngularJoints; ++joint) {
    if (!joint_interfaces_[joint].command_handle->set_value(commands[joint])) {
      return false;
    }
  }
  return true;
}

void InvertedPendulumController::reset() {
  logic_.deactivate();
  joint_interfaces_ = {};
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
