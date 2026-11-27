#include <algorithm>
#include <atomic>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/introspection.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <riser_motion/trajectory.hpp>
#include <riser_motion/srv/jog.hpp>

namespace riser_motion {
class MotionController final
    : public controller_interface::ControllerInterface {
  Settings settings_;
  MotorProfile profile_;
  TrajectoryPlayback playback_;
  double hold_position_ = NAN;
  double command_output_ = NAN, playback_active_ = 0;
  rclcpp::Clock request_clock_{RCL_SYSTEM_TIME};
  uint64_t consumed_ = 0;
  bool angular_ = false, position_mode_ = false;
  realtime_tools::RealtimeBuffer<Trajectory> trajectory_;
  std::string joint_;
  std::string command_name_;
  std::vector<std::string> state_names_;
  bool use_acquisition_time_ = false;
  double last_acquisition_s_ = NAN;
  double lower_position_ = 0, upper_position_ = 0;
  std::atomic<uint64_t> activation_{0}, occupied_{0};
  uint64_t sequence_ = 0;
  int64_t latest_stamp_ = 0;
  bool jog_mode_ = false;
  double speed_cap_ = 0, duration_cap_ = 0;
  struct Heartbeat {
    int64_t token = 0;
    JogClock::time_point time;
  };
  JogProfile jog_;
  realtime_tools::RealtimeBuffer<JogRequest> requests_;
  realtime_tools::RealtimeBuffer<Heartbeat> heartbeat_;
  rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr heartbeat_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_guard_;
  rclcpp::Service<srv::Jog>::SharedPtr service_;
  MotorSample sample() const {
    const double ready = angular_ ? 2 : state_interfaces_[2].get_optional(1).value_or(NAN);
    const bool valid_feedback =
        !jog_mode_ || state_interfaces_[angular_ ? 2 : 5].get_optional(1).value_or(0) == 1;
    return {state_interfaces_[0].get_optional(1).value_or(NAN),
            state_interfaces_[1].get_optional(1).value_or(NAN),
            valid_feedback ? ready : 0};
  }

public:
  controller_interface::CallbackReturn on_init() override {
    parameter_guard_ =
        get_node()->add_on_set_parameters_callback([this](const auto &) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = !(activation_.load() & 1);
          result.reason = result.successful
                              ? ""
                              : "Deactivate and reconfigure before editing";
          return result;
        });
    auto_declare<bool>("jog_mode", false);
    auto_declare<std::string>("command_mode", "velocity");
    auto_declare<std::string>("read_time_interface", "");
    auto_declare<double>("max_velocity_m_s", 0.0);
    auto_declare<double>("max_velocity_rad_s", 0.0);
    auto_declare<double>("max_duration_s", 0.0);
    auto_declare<std::string>("joint", "bottom_motor");
    auto_declare<std::string>("profile", "ramp");
    auto_declare<double>("speed_m_s", 0.0);
    auto_declare<double>("duration_s", 4);
    auto_declare<double>("ramp_s", 1);
    auto_declare<double>("max_travel_m", 0.0);
    auto_declare<double>("max_acceleration_m_s2", 0.0);
    auto_declare<double>("footplate_lower_rad", -0.18);
    auto_declare<double>("footplate_upper_rad", 0.18);
    auto_declare<double>("max_period_s", 0.005);
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override {
    joint_ = get_node()->get_parameter("joint").as_string();
    const bool footplate = joint_ == "foot_plate_pitch";
    angular_ = footplate || joint_ == "backboard_pitch" ||
               joint_ == "ml_angle";
    if (!angular_ && (joint_.size() < 6 || joint_.compare(joint_.size() - 6, 6, "_motor") != 0))
      return controller_interface::CallbackReturn::ERROR;
    const auto command = get_node()->get_parameter("command_mode").as_string();
    if (command != "velocity" && command != "position")
      return controller_interface::CallbackReturn::ERROR;
    position_mode_ = command == "position";
    command_name_ = joint_ + "/" + command;
    settings_.profile = get_node()->get_parameter("profile").as_string();
    settings_.speed = get_node()->get_parameter("speed_m_s").as_double();
    settings_.duration = get_node()->get_parameter("duration_s").as_double();
    settings_.ramp = get_node()->get_parameter("ramp_s").as_double();
    settings_.max_travel =
        get_node()->get_parameter("max_travel_m").as_double();
    settings_.max_acceleration =
        get_node()->get_parameter("max_acceleration_m_s2").as_double();
    settings_.max_period =
        get_node()->get_parameter("max_period_s").as_double();
    jog_mode_ = get_node()->get_parameter("jog_mode").as_bool();
    if (angular_ || position_mode_)
      jog_mode_ = true;
    state_names_ = {joint_ + "/position", joint_ + "/velocity",
                    joint_ + (angular_ ? "/valid" : "/ready")};
    if (jog_mode_) {
      state_names_.push_back(joint_ + "/reverse_limit");
      state_names_.push_back(joint_ + "/forward_limit");
      if (!angular_)
        state_names_.push_back(joint_ + "/feedback_valid");
    }
    const auto read_time = get_node()->get_parameter("read_time_interface").as_string();
    use_acquisition_time_ = !read_time.empty();
    if (use_acquisition_time_)
      state_names_.push_back(read_time);
    const char *speed_cap_parameter =
        angular_ ? "max_velocity_rad_s" : "max_velocity_m_s";
    const double requested_speed_cap =
        get_node()->get_parameter(speed_cap_parameter).as_double();
    if (!std::isfinite(requested_speed_cap) || requested_speed_cap < 0) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid speed cap for %s",
                   joint_.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    if (!configure_limits(position_mode_ || footplate, requested_speed_cap))
      return controller_interface::CallbackReturn::ERROR;
    settings_.position_envelope.reset();
    if (footplate) {
      const PositionEnvelope envelope{
          get_node()->get_parameter("footplate_lower_rad").as_double(),
          get_node()->get_parameter("footplate_upper_rad").as_double()};
      if (!(envelope.lower < envelope.upper &&
            envelope.lower >= lower_position_ && envelope.upper <= upper_position_)) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Footplate profile bounds must be ordered and within [%g, %g] rad",
                     lower_position_, upper_position_);
        return controller_interface::CallbackReturn::ERROR;
      }
      settings_.position_envelope = envelope;
    }
    if (jog_mode_)
      settings_.profile = "ramp"; // A signed jog never reverses mid-request.
    settings_.feedback_speed_tolerance =
        angular_ ? 10 * kSpeedToleranceMps : kSpeedToleranceMps;
    duration_cap_ = get_node()->get_parameter("max_duration_s").as_double();
    if (joint_.empty() || !std::isfinite(duration_cap_) || duration_cap_ <= 0 ||
        std::abs(settings_.speed) > speed_cap_ ||
        settings_.duration > duration_cap_ || !valid(settings_)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Invalid profile, acceleration, timing or travel budget");
      return controller_interface::CallbackReturn::ERROR;
    }
    REGISTER_ROS2_CONTROL_INTROSPECTION("command", &command_output_)
    REGISTER_ROS2_CONTROL_INTROSPECTION("playback_active", &playback_active_)
    if (jog_mode_) {
      heartbeat_sub_ =
          get_node()->create_subscription<builtin_interfaces::msg::Time>(
              "~/jog_heartbeat", rclcpp::QoS(1).best_effort(),
              [this](const builtin_interfaces::msg::Time &stamp) {
                heartbeat_.writeFromNonRT(
                    {static_cast<int64_t>(stamp.sec) * 1000000000 +
                         stamp.nanosec,
                     JogClock::now()});
              });
      service_ = get_node()->create_service<srv::Jog>(
          "~/jog", [this](const srv::Jog::Request::SharedPtr input,
                          srv::Jog::Response::SharedPtr output) {
            JogRequest request;
            request.activation = activation_.load();
            if (!input->preview && !(request.activation & 1)) {
              output->message = "Activate the controller before jogging";
              return;
            }
            request.token =
                static_cast<int64_t>(input->stamp.sec) * 1000000000 +
                input->stamp.nanosec;
            const auto received = JogClock::now();
            const int64_t now_ns = request_clock_.now().nanoseconds();
            const auto age = std::chrono::nanoseconds(now_ns - request.token);
            auto settings = settings_;
            settings.speed = input->velocity_m_s;
            settings.duration = input->duration_s;
            request.cancel = input->cancel;
            const bool custom = !input->csv.empty() || !input->profile.empty();
            Trajectory plan;
            if (!request.cancel) {
              if (!input->preview) {
                if (input->stamp.nanosec >= 1000000000) {
                  output->message = "Malformed request stamp";
                  return;
                }
                output->message = jog_stamp_error(age, request.token, latest_stamp_);
                if (!output->message.empty())
                  return;
              }
              try {
                if (!input->csv.empty()) {
                  plan.samples = parse_trajectory(input->csv, position_mode_,
                                                   settings_, speed_cap_, duration_cap_);
                  if (position_mode_ || settings_.position_envelope) {
                    plan.measure_position_offsets(position_mode_);
                    const auto envelope = settings_.position_envelope.value_or(
                        PositionEnvelope{lower_position_, upper_position_});
                    if (plan.max_offset - plan.min_offset > envelope.upper - envelope.lower)
                      throw std::invalid_argument("CSV position excursion exceeds the position envelope");
                  }
                } else if (position_mode_) {
                  throw std::invalid_argument("Position mode requires a position CSV");
                } else {
                  if (custom) {
                    settings.profile = input->profile;
                    settings.ramp = input->ramp_s;
                  }
                  output->message = jog_limits_error(settings, speed_cap_, duration_cap_);
                  if (!output->message.empty())
                    return;
                }
              } catch (const std::invalid_argument &error) {
                output->message = error.what();
                return;
              }
              plan.settings = settings;
            }
            if (input->preview) {
              output->accepted = !request.cancel;
              output->message = std::to_string(plan.samples.size()) +
                                " CSV samples, duration " +
                                std::to_string(plan.duration()) + " s";
              return;
            }
            request.id = ++sequence_;
            if (!request.cancel) {
              uint64_t idle = 0;
              if (!occupied_.compare_exchange_strong(idle, request.id)) {
                output->message =
                    "Jog already running; cancel before starting another";
                return;
              }
            }
            if (custom && !request.cancel) {
              request.trajectory = true;
              trajectory_.writeFromNonRT(plan);
            }
            request.velocity = settings.speed;
            request.duration = settings.duration;
            latest_stamp_ = request.cancel ? std::max(latest_stamp_, now_ns)
                                           : request.token;
            request.expires = received +
                              std::chrono::duration_cast<JogClock::duration>(
                                  kJogRequestLifetime -
                                  std::max(age, std::chrono::nanoseconds::zero()));
            // Keep the slot occupied until the realtime loop processes cancel.
            if (request.cancel)
              occupied_.store(request.id);
            requests_.writeFromNonRT(request);
            output->accepted = true;
            output->message = "Queued; the control cycle checks live feedback";
          });
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  bool configure_limits(bool need_position, double requested_speed_cap) {
    try {
      const auto components = hardware_interface::parse_control_resources_from_urdf(
          get_robot_description());
      for (const auto &component : components) {
        for (const auto *resources : {&component.joints, &component.gpios}) {
          for (const auto &resource : *resources) {
            if (resource.name != joint_)
              continue;
            const double master_speed =
                std::stod(resource.parameters.at("velocity_limit"));
            if (!std::isfinite(master_speed) || master_speed <= 0)
              break;
            speed_cap_ = requested_speed_cap == 0
                             ? master_speed
                             : std::min(requested_speed_cap, master_speed);
            if (!need_position)
              return true;
            lower_position_ = std::stod(resource.parameters.at("lower"));
            upper_position_ = std::stod(resource.parameters.at("upper"));
            if (std::isfinite(lower_position_) && std::isfinite(upper_position_) &&
                lower_position_ < upper_position_)
              return true;
          }
        }
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
    }
    RCLCPP_ERROR(get_node()->get_logger(), "Missing or invalid motion limits for %s",
                 joint_.c_str());
    return false;
  }

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {command_name_}};
  }
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            state_names_};
  }
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override {
    if (command_interfaces_.size() != 1 ||
        state_interfaces_.size() != state_names_.size() ||
        command_interfaces_[0].get_name() != command_name_)
      return controller_interface::CallbackReturn::ERROR;
    for (std::size_t i = 0; i < state_names_.size(); ++i)
      if (state_interfaces_[i].get_name() != state_names_[i])
        return controller_interface::CallbackReturn::ERROR;
    const auto feedback = sample();
    if (use_acquisition_time_)
      last_acquisition_s_ = state_interfaces_.back().get_optional(1).value_or(NAN);
    hold_position_ = feedback.position;
    if (!profile_.start(settings_, feedback) ||
        !command_interfaces_[0].set_value(position_mode_ ? hold_position_ : 0., 1))
      return controller_interface::CallbackReturn::ERROR;
    command_output_ = position_mode_ ? hold_position_ : 0.;
    playback_active_ = 0;
    playback_.stop();
    consumed_ = requests_.readFromRT()->id;
    jog_.stop();
    occupied_.store(0);
    ++activation_;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override {
    ++activation_;
    playback_active_ = 0;
    command_output_ = position_mode_ ? hold_position_ : 0.;
    if (!command_interfaces_.empty() &&
        !command_interfaces_[0].set_value(position_mode_ ? hold_position_ : 0., 1))
      return controller_interface::CallbackReturn::ERROR;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::return_type
  update(const rclcpp::Time &, const rclcpp::Duration &period) override {
    const auto feedback = sample();
    double dt = period.seconds();
    if (use_acquisition_time_) {
      // Gazebo records the physics time alongside the feedback. Use it
      // directly rather than deriving playback time from asynchronous /clock.
      const double current = state_interfaces_.back().get_optional(1).value_or(NAN);
      dt = current - last_acquisition_s_;
      last_acquisition_s_ = current;
    }
    std::optional<double> target{0.};
    if (jog_mode_) {
      const auto beat = *heartbeat_.readFromRT();
      const auto request = *requests_.readFromRT();
      const double negative =
          state_interfaces_[3].get_optional(1).value_or(NAN);
      const double positive =
          state_interfaces_[4].get_optional(1).value_or(NAN);
      const auto now = JogClock::now();
      const bool healthy = feedback.ready == 2 && std::isfinite(feedback.position) &&
                           std::isfinite(feedback.velocity);
      if (request.id != consumed_) {
        consumed_ = request.id;
        if (request.cancel) {
          playback_.stop();
          if (healthy)
            hold_position_ = feedback.position;
        } else if (request.trajectory && request.activation == activation_.load()) {
          const auto *plan = trajectory_.readFromRT();
          // A loaded position servo can have steady tracking error. Keep the
          // held setpoint as the origin to avoid stepping the command at Start.
          const double origin = position_mode_ ? hold_position_ : feedback.position;
          const auto envelope = settings_.position_envelope.value_or(
              PositionEnvelope{lower_position_, upper_position_});
          if ((!position_mode_ && !settings_.position_envelope) ||
              envelope.contains(origin, plan->min_offset, plan->max_offset)) {
            // Footplate playback still starts from nearly stationary feedback.
            constexpr double footplate_start_speed_rad_s = 0.0015;
            const double starting_speed =
                settings_.position_envelope ? footplate_start_speed_rad_s : speed_cap_ * 0.05;
            playback_.start(request, now, feedback, starting_speed, origin);
          }
        }
      }
      if (request.trajectory && playback_.running()) {
        const auto *plan = trajectory_.readFromRT();
        target = playback_.update(*plan, position_mode_, feedback, speed_cap_,
                                  dt, negative, positive,
                                  request.token, beat.token, beat.time, now,
                                  use_acquisition_time_);
        if (position_mode_)
          hold_position_ = target.value_or(healthy ? feedback.position : hold_position_);
      } else if (position_mode_) {
        target = hold_position_;
      } else {
        target = jog_.update(settings_, activation_.load(),
                             request.trajectory ? JogRequest{} : request,
                             beat.token, beat.time, now, dt,
                             feedback, negative, positive, use_acquisition_time_);
      }
      if (!jog_.running() && !playback_.running()) {
        auto completed = request.id;
        occupied_.compare_exchange_strong(completed, 0);
      }
      playback_active_ = jog_.running() || playback_.running();
      command_output_ = target.value_or(position_mode_ ? hold_position_ : 0.);
      if (!command_interfaces_[0].set_value(command_output_, 1))
        return controller_interface::return_type::ERROR;
      return position_mode_ && !healthy ? controller_interface::return_type::ERROR
                                       : controller_interface::return_type::OK;
    } else {
      target = profile_.update(dt, feedback);
    }
    command_output_ = target.value_or(0);
    if (!command_interfaces_[0].set_value(command_output_, 1) || !target)
      return controller_interface::return_type::ERROR;
    return controller_interface::return_type::OK;
  }
};
} // namespace riser_motion
PLUGINLIB_EXPORT_CLASS(riser_motion::MotionController,
                       controller_interface::ControllerInterface)
