#include "riser_wrench_aggregation/wrench_aggregation_controller.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <hardware_interface/introspection.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace riser_wrench_aggregation {
using Callback = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;

namespace {
constexpr auto FootPitch = "foot_plate_pitch/position";
constexpr auto SubjectMass = "subject_mass_kg";
constexpr auto TorqueBefore = "torque.x_before_perturbation";
static_assert(std::atomic<double>::is_always_lock_free);

std::vector<double> load_csv(const std::string &path) {
  if (path.empty())
    return {};
  if (!std::filesystem::path(path).is_absolute())
    throw std::runtime_error("perturbation.csv_path must be absolute");
  std::ifstream file(path);
  if (!file)
    throw std::runtime_error("Cannot open perturbation CSV: " + path);
  std::vector<double> samples;
  for (std::string line; std::getline(file, line);) {
    try {
      std::size_t consumed = 0;
      const double value = std::stod(line, &consumed);
      if (line.find_first_not_of(" \t\r", consumed) != std::string::npos ||
          !std::isfinite(value))
        throw std::invalid_argument("invalid torque sample");
      samples.push_back(value);
    } catch (const std::logic_error &) {
      throw std::runtime_error("Invalid perturbation CSV sample at line " +
                               std::to_string(samples.size() + 1));
    }
  }
  if (file.bad() || samples.empty())
    throw std::runtime_error("Perturbation CSV is empty or unreadable: " +
                             path);
  return samples;
}
} // namespace

controller_interface::InterfaceConfiguration
WrenchAggregationController::state_interface_configuration() const {
  std::vector<std::string> names;
  for (std::size_t i = 0; i < plate_count_; ++i)
    for (const auto *interface : PlateInterfaceNames)
      names.push_back(plate_names_[i] + "/" + interface);
  if (!read_time_interface_name_.empty())
    names.push_back(read_time_interface_name_);
  names.push_back(FootPitch);
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
  interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
      get_name(), SubjectMass, &subject_mass_state_));
  interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
      get_name(), TorqueBefore, &logic_.torque_x_before_perturbation()));
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
    auto_declare("right_plate_enabled", true);
    const logic::PerturbationParameters perturbation;
    auto_declare("perturbation.multisine_peak_nm",
                 perturbation.multisine_peak_nm);
    auto_declare("perturbation.multisine_low_hz",
                 perturbation.multisine_low_hz);
    auto_declare("perturbation.multisine_high_hz",
                 perturbation.multisine_high_hz);
    auto_declare("perturbation.csv_sample_period_s",
                 perturbation.csv_sample_period_s);
    auto_declare("perturbation.csv_path", std::string{});
    const auto node = get_node();
    rcl_interfaces::msg::ParameterDescriptor mass_descriptor;
    mass_descriptor.description =
        "Measured participant mass in kg (0 until measured)";
    node->declare_parameter(SubjectMass, 0., mass_descriptor, true);
    parameter_guard_ = node->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto &parameter : parameters) {
            if (parameter.get_name() == SubjectMass) {
              if (parameter.get_type() ==
                      rclcpp::ParameterType::PARAMETER_DOUBLE &&
                  parameter.as_double() == measured_mass_kg_.load())
                continue;
              result.reason = "Use measure_mass to update the measured mass";
            } else if (active_.load()) {
              result.reason =
                  "Deactivate and reconfigure before editing parameters";
            } else {
              continue;
            }
            result.successful = false;
            break;
          }
          return result;
        });
    using Tare = std_srvs::srv::SetBool;
    using Mass = std_srvs::srv::Trigger;
    play_csv_service_ = node->create_service<Mass>(
        "~/play_perturbation", [this](const Mass::Request::SharedPtr,
                                      Mass::Response::SharedPtr response) {
          if (!active_.load() || !csv_configured_.load()) {
            response->message = "Activate the controller with a configured CSV";
            return;
          }
          auto idle = Playback::Idle;
          response->success =
              playback_.compare_exchange_strong(idle, Playback::Requested);
          response->message = response->success
                                  ? "CSV queued for the next update"
                                  : "CSV playback is already busy";
        });
    multisine_service_ = node->create_service<Tare>(
        "~/set_multisine", [this](const Tare::Request::SharedPtr request,
                                  Tare::Response::SharedPtr response) {
          if (!active_.load() ||
              (request->data && !multisine_configured_.load())) {
            response->message =
                "Activate the controller with a nonzero multisine peak";
            return;
          }
          multisine_enabled_.store(request->data);
          response->success = true;
          response->message =
              request->data ? "Multisine enabled" : "Multisine disabled";
        });
    tare_service_ = node->create_service<Tare>(
        "~/tare", [this](std::shared_ptr<rmw_request_id_t> header,
                         std::shared_ptr<Tare::Request> request) {
          request_capture(
              request->data ? logic::Capture::EmptyTare
                            : logic::Capture::StandingTare,
              [this, header](bool success, const std::string &message) {
                Tare::Response response;
                response.success = success;
                response.message = message;
                tare_service_->send_response(*header, response);
              });
        });
    mass_service_ = node->create_service<Mass>(
        "~/measure_mass", [this](std::shared_ptr<rmw_request_id_t> header,
                                 std::shared_ptr<Mass::Request>) {
          request_capture(
              logic::Capture::Mass,
              [this, header](bool success, const std::string &message) {
                Mass::Response response;
                response.success = success;
                response.message = message;
                mass_service_->send_response(*header, response);
              });
        });
    capture_timer_ = node->create_wall_timer(std::chrono::milliseconds(20),
                                             [this] { respond_to_capture(); });
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
    return Callback::ERROR;
  }
  return Callback::SUCCESS;
}

void WrenchAggregationController::request_capture(logic::Capture mode,
                                                  CaptureReply reply) {
  if (!active_.load()) {
    reply(false, "Ankle wrench controller is inactive");
  } else if (capture_reply_) {
    reply(false, "A capture is already in progress");
  } else {
    capture_reply_ = std::move(reply);
    capture_result_.store(logic::CaptureResult::Pending);
    capture_request_.store(mode);
  }
}

void WrenchAggregationController::respond_to_capture() {
  using logic::CaptureResult;
  auto result = capture_result_.load();
  const double mass_kg = measured_mass_kg_.load();
  if (parameter_mass_kg_ != mass_kg) {
    if (!get_node()
             ->set_parameter(rclcpp::Parameter(SubjectMass, mass_kg))
             .successful)
      return; // The measurement changed between the snapshot and parameter
              // guard.
    parameter_mass_kg_ = mass_kg;
  }

  if (!capture_reply_)
    return;
  if (result == CaptureResult::Pending) {
    if (active_.load())
      return;
    capture_request_.store(logic::Capture::None);
    result = CaptureResult::Cancelled;
  }

  const char *message = "Capture complete";
  switch (result) {
  case CaptureResult::Pending:
  case CaptureResult::Success:
    break;
  case CaptureResult::InvalidSample:
    message = "Capture lost valid force-plate samples";
    break;
  case CaptureResult::NeedsEmptyTare:
    message = "Tare the empty plates before measuring mass";
    break;
  case CaptureResult::PlateNotFlat:
    message = "Footplate must be flat throughout the capture";
    break;
  case CaptureResult::NoLoad:
    message = "No measurable load above the empty-plate baseline";
    break;
  case CaptureResult::Cancelled:
    message = "Controller deactivated during capture";
    break;
  }
  auto reply = std::move(capture_reply_);
  capture_reply_ = {};
  reply(result == CaptureResult::Success, message);
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
    logic::PerturbationParameters perturbation;
    perturbation.multisine_peak_nm =
        node->get_parameter("perturbation.multisine_peak_nm").as_double();
    perturbation.multisine_low_hz =
        node->get_parameter("perturbation.multisine_low_hz").as_double();
    perturbation.multisine_high_hz =
        node->get_parameter("perturbation.multisine_high_hz").as_double();
    perturbation.csv_sample_period_s =
        node->get_parameter("perturbation.csv_sample_period_s").as_double();
    perturbation.csv =
        load_csv(node->get_parameter("perturbation.csv_path").as_string());
    const bool csv_configured = !perturbation.csv.empty();
    const bool multisine_configured = perturbation.multisine_peak_nm > 0.;
    if (const auto *error =
            logic_.configure(parameters, std::move(perturbation))) {
      RCLCPP_ERROR(node->get_logger(), "%s", error);
      return Callback::ERROR;
    }
    plate_count_ = node->get_parameter("right_plate_enabled").as_bool() ? 2 : 1;
    read_time_interface_name_ =
        node->get_parameter("read_time_interface").as_string();
    csv_configured_.store(csv_configured);
    multisine_configured_.store(multisine_configured);
    logic_.register_introspection(
        [this](const char *name, const double *value) {
          REGISTER_ROS2_CONTROL_INTROSPECTION(name, value)
        });
    subject_mass_state_ = 0.;
    measured_mass_kg_.store(subject_mass_state_);
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
  for (std::size_t plate = 0; plate < plate_count_; ++plate) {
    for (std::size_t component = 0; component < PlateInterfaceNames.size();
         ++component) {
      const auto name =
          plate_names_[plate] + "/" + PlateInterfaceNames[component];
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
    const auto found =
        std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                     [&](const auto &interface) {
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
  const auto foot_pitch = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [](const auto &interface) { return interface.get_name() == FootPitch; });
  if (foot_pitch == state_interfaces_.end()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing state interface %s",
                 FootPitch);
    reset();
    return Callback::ERROR;
  }
  foot_pitch_interface_ = &*foot_pitch;
  if (const auto *error = logic_.activate()) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error);
    reset();
    return Callback::ERROR;
  }
  active_.store(true);
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
                                              logic::PlateSample &sample) {
  std::array<double, PlateInterfaceNames.size()> values{};
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (!interfaces.handles[i])
      return false;
    const auto value = interfaces.handles[i]->get_optional(1);
    if (!value)
      return false;
    values[i] = *value;
  }
  if (values.back() != 0. && values.back() != 1.)
    return false;
  std::copy_n(values.begin(), 3, sample.wrench.force.begin());
  std::copy_n(values.begin() + 3, 3, sample.wrench.torque.begin());
  sample.valid = values.back() == 1.;
  return true;
}

bool WrenchAggregationController::read_inputs(logic::Inputs &inputs) const {
  if (plate_count_ == 1)
    inputs.right.valid = true; // An omitted plate contributes zero wrench.
  return read_sample(plate_interfaces_[0], inputs.left) &&
         (plate_count_ == 1 || read_sample(plate_interfaces_[1], inputs.right));
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
  active_.store(false);
  multisine_enabled_.store(false);
  playback_.store(Playback::Idle);
  const auto queued = capture_request_.exchange(logic::Capture::None);
  if (queued != logic::Capture::None || logic_.capture_active())
    capture_result_.store(logic::CaptureResult::Cancelled);
  logic_.deactivate();
  plate_interfaces_ = {};
  read_time_interface_ = nullptr;
  foot_pitch_interface_ = nullptr;
  publish_sample({});
}

Return WrenchAggregationController::update_reference_from_subscribers(
    const rclcpp::Time &, const rclcpp::Duration &) {
  return Return::OK;
}

Return WrenchAggregationController::update_and_write_commands(
    const rclcpp::Time &time, const rclcpp::Duration &) {
  logic::Inputs inputs;
  if (!read_inputs(inputs))
    inputs = {};
  const double read_time_s =
      read_time_interface_
          ? read_time_interface_->get_optional(1).value_or(logic::Unavailable)
          : time.seconds();
  if (capture_request_.load() != logic::Capture::None) {
    const auto request = capture_request_.exchange(logic::Capture::None);
    if (request != logic::Capture::None)
      logic_.begin_capture(request);
  }
  const double foot_pitch_rad =
      logic_.capture_needs_flat_plate()
          ? foot_pitch_interface_->get_optional(1).value_or(logic::Unavailable)
          : logic::Unavailable;
  const auto playback = playback_.load();
  if (playback == Playback::Requested)
    playback_.store(Playback::Running);
  const auto result =
      logic_.update(inputs, read_time_s, foot_pitch_rad,
                    multisine_enabled_.load(), playback == Playback::Requested);
  if (playback != Playback::Idle && !logic_.csv_playing())
    playback_.store(Playback::Idle);
  publish_sample(result.output);
  if (result.capture != logic::CaptureResult::Pending) {
    subject_mass_state_ = logic_.subject_mass_kg();
    measured_mass_kg_.store(subject_mass_state_);
    capture_result_.store(result.capture);
  }
  return Return::OK;
}
} // namespace riser_wrench_aggregation

PLUGINLIB_EXPORT_CLASS(riser_wrench_aggregation::WrenchAggregationController,
                       controller_interface::ChainableControllerInterface)
