#include <atomic>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <riser_motion_bringup/jog.hpp>
#include <riser_motion_bringup/srv/jog.hpp>

namespace riser_motion_bringup {
class MotionController final
    : public controller_interface::ControllerInterface {
  Settings settings_;
  MotorProfile profile_;
  std::string joint_;
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
    const double ready = state_interfaces_[2].get_optional(1).value_or(NAN);
    const bool valid_feedback =
        !jog_mode_ || state_interfaces_[5].get_optional(1).value_or(0) == 1;
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
    auto_declare<double>("max_velocity_rad_s", 10.0);
    auto_declare<double>("max_duration_s", 10.);
    auto_declare<std::string>("joint", "bottom_motor");
    auto_declare<std::string>("profile", "ramp");
    auto_declare<double>("speed_rad_s", 0.05);
    auto_declare<double>("duration_s", 4);
    auto_declare<double>("ramp_s", 1);
    auto_declare<double>("max_travel_rad", 15.0);
    auto_declare<double>("max_acceleration_rad_s2", 15.0);
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
    jog_mode_ = get_node()->get_parameter("jog_mode").as_bool();
    if (jog_mode_)
      settings_.profile = "ramp"; // A signed jog never reverses mid-request.
    speed_cap_ = get_node()->get_parameter("max_velocity_rad_s").as_double();
    duration_cap_ = get_node()->get_parameter("max_duration_s").as_double();
    if (joint_.empty() || !std::isfinite(speed_cap_) ||
        !std::isfinite(duration_cap_) || speed_cap_ <= 0 ||
        duration_cap_ <= 0 || std::abs(settings_.speed) > speed_cap_ ||
        settings_.duration > duration_cap_ || !valid(settings_)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Invalid profile, acceleration, timing or travel budget");
      return controller_interface::CallbackReturn::ERROR;
    }
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
            if (!(request.activation & 1)) {
              output->message = "Activate the controller before jogging";
              return;
            }
            request.token =
                static_cast<int64_t>(input->stamp.sec) * 1000000000 +
                input->stamp.nanosec;
            const double age = get_node()->now().seconds() -
                               (input->stamp.sec + input->stamp.nanosec * 1e-9);
            auto settings = settings_;
            settings.speed = input->velocity_rad_s;
            settings.duration = input->duration_s;
            request.cancel = input->cancel;
            if (!request.cancel &&
                (input->stamp.nanosec >= 1000000000 ||
                 request.token <= latest_stamp_ || age < 0 || age > 0.5 ||
                 !valid(settings) || std::abs(settings.speed) > speed_cap_ ||
                 settings.duration > duration_cap_)) {
              output->message = "Expired timestamp or speed, duration, ramp or "
                                "travel cap exceeded";
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
            request.velocity = settings.speed;
            request.duration = settings.duration;
            latest_stamp_ = request.cancel ? get_node()->now().nanoseconds()
                                           : request.token;
            request.expires = JogClock::now() +
                              std::chrono::duration_cast<JogClock::duration>(
                                  std::chrono::duration<double>(0.5 - age));
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
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {joint_ + "/velocity"}};
  }
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override {
    std::vector<std::string> states{joint_ + "/position", joint_ + "/velocity",
                                    joint_ + "/ready"};
    if (jog_mode_) {
      states.push_back(joint_ + "/reverse_limit");
      states.push_back(joint_ + "/forward_limit");
      states.push_back(joint_ + "/feedback_valid");
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            states};
  }
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override {
    if (command_interfaces_.size() != 1 ||
        state_interfaces_.size() != (jog_mode_ ? 6U : 3U) ||
        command_interfaces_[0].get_name() != joint_ + "/velocity" ||
        state_interfaces_[0].get_name() != joint_ + "/position" ||
        state_interfaces_[1].get_name() != joint_ + "/velocity" ||
        state_interfaces_[2].get_name() != joint_ + "/ready" ||
        (jog_mode_ &&
         (state_interfaces_[3].get_name() != joint_ + "/reverse_limit" ||
          state_interfaces_[4].get_name() != joint_ + "/forward_limit" ||
          state_interfaces_[5].get_name() != joint_ + "/feedback_valid")) ||
        !profile_.start(settings_, sample()) ||
        !command_interfaces_[0].set_value(0.0, 1))
      return controller_interface::CallbackReturn::ERROR;
    jog_.stop();
    occupied_.store(0);
    ++activation_;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override {
    ++activation_;
    if (!command_interfaces_.empty() &&
        !command_interfaces_[0].set_value(0.0, 1))
      return controller_interface::CallbackReturn::ERROR;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::return_type
  update(const rclcpp::Time &, const rclcpp::Duration &period) override {
    const auto feedback = sample();
    std::optional<double> target{0.};
    if (jog_mode_) {
      const auto beat = *heartbeat_.readFromRT();
      const auto request = *requests_.readFromRT();
      const double negative =
          state_interfaces_[3].get_optional(1).value_or(NAN);
      const double positive =
          state_interfaces_[4].get_optional(1).value_or(NAN);
      target = jog_.update(settings_, activation_.load(), request, beat.token,
                           beat.time, JogClock::now(), period.seconds(),
                           feedback, negative, positive);
      if (!jog_.running()) {
        auto completed = request.id;
        occupied_.compare_exchange_strong(completed, 0);
      }
    } else {
      target = profile_.update(period.seconds(), feedback);
    }
    if (!command_interfaces_[0].set_value(target.value_or(0), 1) || !target)
      return controller_interface::return_type::ERROR;
    return controller_interface::return_type::OK;
  }
};
} // namespace riser_motion_bringup
PLUGINLIB_EXPORT_CLASS(riser_motion_bringup::MotionController,
                       controller_interface::ControllerInterface)
