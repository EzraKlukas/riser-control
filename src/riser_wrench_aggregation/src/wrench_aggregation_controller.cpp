#include "riser_wrench_aggregation/wrench_aggregation_controller.hpp"
#include <algorithm>
#include <pluginlib/class_list_macros.hpp>

namespace riser_wrench_aggregation {
using Callback = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;

controller_interface::InterfaceConfiguration
WrenchAggregationController::state_interface_configuration() const {
  std::vector<std::string> names;
  for (const auto &plate : plate_names_)
    for (const auto *interface : WrenchInterfaceNames)
      names.push_back(plate + "/" + interface);
  if (!read_time_interface_name_.empty())
    names.push_back(read_time_interface_name_);
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          names};
}

controller_interface::InterfaceConfiguration
WrenchAggregationController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE, {}};
}

std::vector<hardware_interface::StateInterface::SharedPtr>
WrenchAggregationController::on_export_state_interfaces_list() {
  std::vector<hardware_interface::StateInterface::SharedPtr> interfaces;
  for (std::size_t i = 0; i < output_values_.size(); ++i)
    interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
        get_name(), WrenchInterfaceNames[i], &output_values_[i]));
  return interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
WrenchAggregationController::on_export_reference_interfaces_list() {
  return {};
}

Callback WrenchAggregationController::on_init() {
  reset();
  try {
    const logic::Parameters defaults;
    auto_declare("ankle_to_left_plate_origin", defaults.left.ankle_to_origin);
    auto_declare("ankle_to_right_plate_origin", defaults.right.ankle_to_origin);
    auto_declare("same_transformation", defaults.same_transformation);
    auto_declare("same_scaling", defaults.same_scaling);
    auto_declare("custom_transformation", defaults.custom_transformation);
    auto_declare("matrix_transformation", defaults.matrix_transformation);
    auto_declare("scaling", defaults.scaling);
    auto_declare("left_wrench_gain", defaults.left.gain);
    auto_declare("right_wrench_gain", defaults.right.gain);
    auto_declare("ankle_wrench_gain", defaults.ankle_gain);
    auto_declare("symmetric_wrench_transformation",
                 defaults.symmetric_transform);
    auto_declare("left_wrench_transformation", defaults.left.linear_transform);
    auto_declare("right_wrench_transformation",
                 defaults.right.linear_transform);
    auto_declare("read_time_interface", std::string{});
    auto_declare("max_sample_age_s", defaults.max_sample_age_s);
    auto_declare("max_sample_skew_s", defaults.max_sample_skew_s);
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
    return Callback::ERROR;
  }
  return Callback::SUCCESS;
}

Callback
WrenchAggregationController::on_configure(const rclcpp_lifecycle::State &) {
  try {
    logic::Parameters parameters;
    const auto node = get_node();
    parameters.left.ankle_to_origin =
        node->get_parameter("ankle_to_left_plate_origin").as_double_array();
    parameters.right.ankle_to_origin =
        node->get_parameter("ankle_to_right_plate_origin").as_double_array();
    parameters.left.linear_transform =
        node->get_parameter("left_wrench_transformation").as_double_array();
    parameters.right.linear_transform =
        node->get_parameter("right_wrench_transformation").as_double_array();
    parameters.symmetric_transform =
        node->get_parameter("symmetric_wrench_transformation")
            .as_double_array();
    parameters.left.gain = node->get_parameter("left_wrench_gain").as_double();
    parameters.right.gain =
        node->get_parameter("right_wrench_gain").as_double();
    parameters.ankle_gain =
        node->get_parameter("ankle_wrench_gain").as_double();
    parameters.same_transformation =
        node->get_parameter("same_transformation").as_bool();
    parameters.same_scaling = node->get_parameter("same_scaling").as_bool();
    parameters.custom_transformation =
        node->get_parameter("custom_transformation").as_bool();
    parameters.matrix_transformation =
        node->get_parameter("matrix_transformation").as_bool();
    parameters.scaling = node->get_parameter("scaling").as_bool();
    parameters.max_sample_age_s =
        node->get_parameter("max_sample_age_s").as_double();
    parameters.max_sample_skew_s =
        node->get_parameter("max_sample_skew_s").as_double();
    if (const auto *error = logic_.configure(parameters)) {
      RCLCPP_ERROR(node->get_logger(), "%s", error);
      return Callback::ERROR;
    }
    read_time_interface_name_ = node->get_parameter("read_time_interface").as_string();
    publish_sample({});
    return Callback::SUCCESS;
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
    return Callback::ERROR;
  }
}

Callback
WrenchAggregationController::on_activate(const rclcpp_lifecycle::State &) {
  reset();
  active_.store(true);
  for (std::size_t plate = 0; plate < plate_names_.size(); ++plate) {
    for (std::size_t component = 0; component < WrenchInterfaceNames.size();
         ++component) {
      const auto name =
          plate_names_[plate] + "/" + WrenchInterfaceNames[component];
      const auto found = std::find_if(
          state_interfaces_.begin(), state_interfaces_.end(),
          [&](const auto &interface) { return interface.get_name() == name; });
      if (found == state_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Missing state interface %s",
                     name.c_str());
        reset();
        return Callback::ERROR;
      }
      plate_interfaces_[plate].handles[component] = &*found;
    }
  }
  if (!read_time_interface_name_.empty()) {
    const auto found = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(), [&](const auto &interface) {
          return interface.get_name() == read_time_interface_name_;
        });
    if (found == state_interfaces_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing read-time interface %s",
                   read_time_interface_name_.c_str());
      reset();
      return Callback::ERROR;
    }
    read_time_interface_ = &*found;
  }
  if (const auto *error = logic_.activate()) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error);
    reset();
    return Callback::ERROR;
  }
  return Callback::SUCCESS;
}

Callback
WrenchAggregationController::on_deactivate(const rclcpp_lifecycle::State &) {
  reset();
  return Callback::SUCCESS;
}

Callback
WrenchAggregationController::on_cleanup(const rclcpp_lifecycle::State &) {
  reset();
  return Callback::SUCCESS;
}

bool WrenchAggregationController::read_sample(const PlateInterfaces &interfaces,
                                              logic::WrenchSample &sample) {
  std::array<double, 8> values{};
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (!interfaces.handles[i])
      return false;
    const auto value = interfaces.handles[i]->get_optional(1);
    if (!value)
      return false;
    values[i] = *value;
  }
  if (values[7] != 0. && values[7] != 1.)
    return false;
  std::copy_n(values.begin(), 3, sample.wrench.force.begin());
  std::copy_n(values.begin() + 3, 3, sample.wrench.torque.begin());
  sample.sample_time_s = values[6];
  sample.valid = values[7] == 1.;
  return true;
}

bool WrenchAggregationController::read_inputs(logic::Inputs &inputs) const {
  return read_sample(plate_interfaces_[0], inputs.left) &&
         read_sample(plate_interfaces_[1], inputs.right);
}

void WrenchAggregationController::publish_sample(
    const logic::WrenchSample &sample) {
  std::copy(sample.wrench.force.begin(), sample.wrench.force.end(),
            output_values_.begin());
  std::copy(sample.wrench.torque.begin(), sample.wrench.torque.end(),
            output_values_.begin() + 3);
  output_values_[6] = sample.sample_time_s;
  output_values_[7] = sample.valid ? 1. : 0.;
}

void WrenchAggregationController::reset() {
  logic_.deactivate();
  plate_interfaces_ = {};
  read_time_interface_ = nullptr;
  last_read_time_s_ = logic::Unavailable;
  publish_sample({});
  active_.store(false);
}

Return WrenchAggregationController::update_reference_from_subscribers(
    const rclcpp::Time &, const rclcpp::Duration &) {
  return Return::OK;
}

Return WrenchAggregationController::update_and_write_commands(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
  logic::Inputs inputs;
  if (!read_inputs(inputs)) {
    publish_sample({});
    return Return::OK;
  }
  double read_time_s = time.seconds();
  double elapsed_s = period.seconds();
  if (read_time_interface_) {
    const auto value = read_time_interface_->get_optional(1);
    if (!value) {
      publish_sample({});
      return Return::OK;
    }
    read_time_s = *value;
    elapsed_s = std::isfinite(last_read_time_s_) && read_time_s >= last_read_time_s_
                    ? read_time_s - last_read_time_s_
                    : 0.;
    last_read_time_s_ = read_time_s;
  }
  const auto result = logic_.update(inputs, read_time_s, elapsed_s);
  publish_sample(result.output);
  return Return::OK;
}
} // namespace riser_wrench_aggregation

PLUGINLIB_EXPORT_CLASS(riser_wrench_aggregation::WrenchAggregationController,
                       controller_interface::ChainableControllerInterface)
