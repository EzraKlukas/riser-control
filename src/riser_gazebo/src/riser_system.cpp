// Humble + Fortress. Direct physical pitch torques with removed
// stage/rod dynamics reduced onto pitch.
// No physical pitch-joint position/velocity resets are used.
#include <algorithm>
#include <array>
#include <cmath>
#include <gz_ros2_control/gz_system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <limits>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace riser_gazebo {
using Ret = hardware_interface::return_type;
class AngularSystem final : public gz_ros2_control::GazeboSimSystemInterface {
  sim::EntityComponentManager *ecm_ = nullptr;
  std::array<std::string, 3> names_{
      {"backboard_pitch", "foot_plate_pitch", "ml_angle"}};
  std::array<std::string, 4> physical_{
      {"backboard_pitch", "foot_plate_pitch", "torso_lateral", "hip_lateral"}};
  std::array<sim::Entity, 4> joints_{};
  std::array<double, 3> q_{}, v_{}, effort_{}, cmd_{}, ref_{}, ref_v_{};
  std::array<double, 3> lower_{}, upper_{}, vmax_{};
  std::array<double, 4> motor_force_{};
  std::array<double, 2> heights_{};
  sensor_msgs::msg::JointState physical_msg_;
  int64_t last_publish_ns_ = -1;
  double state_rate_ = 100;
  static constexpr size_t physical_count_ = 4;
  double kp_ = 0, kd_ = 0, max_force_ = 0;
  // Table columns: angle, inertia, inertia derivative, gravity torque, damping.
  struct ReducedSample {
    double q, m, dm, g, d;
  };
  std::array<std::vector<ReducedSample>, 2> reduced_;
  std::array<double, 2> pitch_kp_{}, pitch_kd_{}, pitch_max_{},
      previous_pitch_v_{};
  bool previous_pitch_velocity_valid_ = false;
  ReducedSample reduced_at(size_t i, double q) const {
    const auto &t = reduced_[i];
    const double step = t[1].q - t[0].q;
    const double index =
        std::clamp((q - t.front().q) / step, 0., double(t.size() - 1));
    const size_t j = std::min(size_t(index), t.size() - 2);
    const double f = index - j;
    const auto &a = t[j];
    const auto &b = t[j + 1];
    return {q, a.m + f * (b.m - a.m), a.dm + f * (b.dm - a.dm),
            a.g + f * (b.g - a.g), a.d + f * (b.d - a.d)};
  }
  void apply_direct(double dt) {
    for (size_t i = 0; i < 2; ++i) {
      const double q = state(i), v = state(i, true);
      if (!std::isfinite(q) || !std::isfinite(v))
        throw std::domain_error("Nonfinite direct pitch state");
      const double acceleration =
          previous_pitch_velocity_valid_ ? (v - previous_pitch_v_[i]) / dt : 0.;
      const auto r = reduced_at(i, q);
      // M(q) qdd + 1/2 M'(q) qd^2 + D(q) qd - Qg(q): the
      // complete planar contribution of the removed audited stage and rod.
      // Acceleration feedback is delayed by one physics step. This numerical
      // approximation is quantified in the historical runtime validation.
      force(i, motor_force_[i] - r.m * acceleration - .5 * r.dm * v * v -
                   r.d * v + r.g);
      previous_pitch_v_[i] = v;
    }
    previous_pitch_velocity_valid_ = true;
  }
  bool ready_ = false, active_ = false;
  int64_t last_write_ns_ = -1;
  std::string mode_ = "position";
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  double number(const hardware_interface::HardwareInfo &h,
                const std::string &s) {
    return std::stod(h.hardware_parameters.at(s));
  }
  double state(size_t i, bool vel = false) const {
    if (vel) {
      auto c = ecm_->Component<sim::components::JointVelocity>(joints_[i]);
      if (!c || c->Data().empty())
        throw std::runtime_error("Missing joint velocity");
      return c->Data()[0];
    }
    auto c = ecm_->Component<sim::components::JointPosition>(joints_[i]);
    if (!c || c->Data().empty())
      throw std::runtime_error("Missing joint position");
    return c->Data()[0];
  }
  void force(size_t joint, double f) {
    auto c = ecm_->Component<sim::components::JointForceCmd>(joints_[joint]);
    if (c)
      c->Data() = {f};
    else
      ecm_->CreateComponent(joints_[joint],
                            sim::components::JointForceCmd({f}));
  }
  void zero_motors() {
    for (size_t i = 0; i < physical_count_; ++i)
      force(i, 0);
    motor_force_.fill(0);
  }

public:
  bool initSim(rclcpp::Node::SharedPtr &node,
               std::map<std::string, sim::Entity> &joints,
               const hardware_interface::HardwareInfo &info,
               sim::EntityComponentManager &ecm, int &) override {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
      return false;
    nh_ = node;
    ecm_ = &ecm;
    try {
      mode_ = info.hardware_parameters.at("command_mode");
      if (mode_ != "position" && mode_ != "velocity")
        throw std::runtime_error("Unsupported command mode");
      for (size_t i = 0; i < physical_count_; ++i) {
        joints_[i] = joints.at(physical_[i]);
        if (!ecm.Component<sim::components::JointPosition>(joints_[i]))
          ecm.CreateComponent(joints_[i], sim::components::JointPosition());
        if (!ecm.Component<sim::components::JointVelocity>(joints_[i]))
          ecm.CreateComponent(joints_[i], sim::components::JointVelocity());
      }
      for (size_t i = 0; i < 2; ++i) {
        const std::string p = i == 0 ? "top_" : "bottom_";
        std::istringstream in(info.hardware_parameters.at(p + "reduced_table"));
        ReducedSample r;
        while (in >> r.q >> r.m >> r.dm >> r.g >> r.d) {
          if (!std::isfinite(r.q) || !std::isfinite(r.m) ||
              !std::isfinite(r.dm) || !std::isfinite(r.g) ||
              !std::isfinite(r.d) || r.m < 0 || r.d < 0)
            throw std::runtime_error("Invalid reduced dynamics");
          reduced_[i].push_back(r);
        }
        const auto &t = reduced_[i];
        if (!in.eof() || t.size() < 2)
          throw std::runtime_error("Malformed reduced dynamics table");
        const double step = t[1].q - t[0].q;
        if (step <= 0)
          throw std::runtime_error("Invalid reduced dynamics grid");
        for (size_t j = 1; j < t.size(); ++j)
          if (std::abs(t[j].q - t[j - 1].q - step) > 1e-10)
            throw std::runtime_error("Nonuniform reduced dynamics grid");
        pitch_kp_[i] = number(info, p + "pitch_kp");
        pitch_kd_[i] = number(info, p + "pitch_kd");
        pitch_max_[i] = number(info, p + "pitch_max_torque");
        if (!std::isfinite(pitch_kp_[i]) || !std::isfinite(pitch_kd_[i]) ||
            !std::isfinite(pitch_max_[i]) || pitch_kp_[i] <= 0 ||
            pitch_kd_[i] < 0 || pitch_max_[i] <= 0)
          throw std::runtime_error("Invalid angular servo gains");
      }
      heights_ = {number(info, "torso_height"), number(info, "hip_height")};
      kp_ = number(info, "servo_kp");
      kd_ = number(info, "servo_kd");
      max_force_ = number(info, "servo_max_force");
      if (info.joints.size() != 3)
        throw std::runtime_error("Exactly three public joints required");
      for (size_t i = 0; i < 3; ++i) {
        auto j =
            std::find_if(info.joints.begin(), info.joints.end(),
                         [&](const auto &j) { return j.name == names_[i]; });
        if (j == info.joints.end())
          throw std::runtime_error("Missing public joint");
        lower_[i] = std::stod(j->parameters.at("lower"));
        upper_[i] = std::stod(j->parameters.at("upper"));
        vmax_[i] = std::stod(j->parameters.at("velocity_limit"));
      }
      state_rate_ = number(info, "physical_state_rate");
      if (!std::isfinite(state_rate_) || state_rate_ <= 0)
        throw std::runtime_error("Invalid physical state rate");
      physical_msg_.name.assign(physical_.begin(),
                                physical_.begin() + physical_count_);
      physical_msg_.name.push_back("ml_angle");
      physical_msg_.position.resize(physical_count_ + 1);
      physical_msg_.velocity.resize(physical_count_ + 1);
      state_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(
          "/riser/physical_joint_states", 10);
    } catch (const std::exception &ex) {
      RCLCPP_ERROR(node->get_logger(), "RISER init: %s", ex.what());
      return false;
    }
    return true;
  }
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> out;
    for (size_t i = 0; i < 3; ++i) {
      out.emplace_back(names_[i], "position", &q_[i]);
      out.emplace_back(names_[i], "velocity", &v_[i]);
      out.emplace_back(names_[i], "effort", &effort_[i]);
    }
    return out;
  }
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> out;
    for (size_t i = 0; i < 3; ++i)
      out.emplace_back(names_[i], mode_, &cmd_[i]);
    return out;
  }
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override {
    active_ = true;
    ref_ = q_;
    ref_v_.fill(0);
    cmd_ = mode_ == "position" ? q_ : std::array<double, 3>{};
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override {
    active_ = false;
    zero_motors();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  Ret read(const rclcpp::Time &now, const rclcpp::Duration &) override {
    try {
      q_[0] = state(0);
      q_[1] = state(1);
      v_[0] = state(0, true);
      v_[1] = state(1, true);
      double ht = heights_[0], hh = heights_[1], h2 = ht * ht + hh * hh,
             t = (ht * state(2) + hh * state(3)) / h2;
      q_[2] = std::atan(t);
      v_[2] = (ht * state(2, true) + hh * state(3, true)) / (h2 * (1 + t * t));
      effort_[0] = motor_force_[0];
      effort_[1] = motor_force_[1];
      effort_[2] = (1 + t * t) * (ht * motor_force_[2] + hh * motor_force_[3]);
      if (last_publish_ns_ < 0 ||
          now.nanoseconds() - last_publish_ns_ >= 1e9 / state_rate_) {
        physical_msg_.header.stamp = now;
        for (size_t i = 0; i < physical_count_; ++i) {
          physical_msg_.position[i] = state(i);
          physical_msg_.velocity[i] = state(i, true);
        }
        physical_msg_.position[physical_count_] = q_[2];
        physical_msg_.velocity[physical_count_] = v_[2];
        state_pub_->publish(physical_msg_);
        last_publish_ns_ = now.nanoseconds();
      }
      ready_ = true;
    } catch (const std::exception &ex) {
      if (ready_) {
        RCLCPP_ERROR(nh_->get_logger(), "RISER read: %s", ex.what());
        zero_motors();
        return Ret::ERROR;
      }
      return Ret::OK;
    }
    return Ret::OK;
  }
  Ret write(const rclcpp::Time &now, const rclcpp::Duration &) override {
    // gz_ros2_control passes time since its last controller update, not since
    // its previous physics-step write. Integrate only the actual write delta.
    const auto ns = now.nanoseconds();
    if (last_write_ns_ >= 0 && ns < last_write_ns_)
      previous_pitch_velocity_valid_ = false;
    double dt =
        last_write_ns_ < 0 ? 0. : std::max(0., (ns - last_write_ns_) * 1e-9);
    last_write_ns_ = ns;
    if (!ready_) {
      zero_motors();
      return Ret::OK;
    }
    if (!active_) {
      zero_motors();
      try {
        if (dt > 0)
          apply_direct(dt);
      } catch (const std::exception &ex) {
        RCLCPP_ERROR(nh_->get_logger(), "RISER passive dynamics: %s",
                     ex.what());
        return Ret::ERROR;
      }
      return Ret::OK;
    }
    try {
      if (dt <= 0)
        return Ret::OK;
      for (size_t i = 0; i < 3; ++i) {
        if (!std::isfinite(cmd_[i]))
          throw std::domain_error("Nonfinite command");
        if (mode_ == "position") {
          if (cmd_[i] < lower_[i] || cmd_[i] > upper_[i])
            throw std::domain_error("Angular target outside bounds");
          ref_[i] = cmd_[i];
          ref_v_[i] = 0;
        } else {
          ref_v_[i] = std::clamp(cmd_[i], -vmax_[i], vmax_[i]);
          ref_[i] = std::clamp(ref_[i] + dt * ref_v_[i], lower_[i], upper_[i]);
          if (ref_[i] == lower_[i] || ref_[i] == upper_[i])
            ref_v_[i] = 0;
        }
      }
      for (size_t i = 0; i < 2; ++i)
        motor_force_[i] = pitch_kp_[i] * (ref_[i] - state(i)) +
                          pitch_kd_[i] * (ref_v_[i] - state(i, true));
      double t = std::tan(ref_[2]);
      for (size_t i = 0; i < 2; ++i) {
        motor_force_[i + 2] =
            kp_ * (heights_[i] * t - state(i + 2)) +
            kd_ * (heights_[i] * (1 + t * t) * ref_v_[2] - state(i + 2, true));
      }
      for (size_t i = 0; i < 4; ++i) {
        const double limit = i < 2 ? pitch_max_[i] : max_force_;
        motor_force_[i] = std::clamp(motor_force_[i], -limit, limit);
      }
      apply_direct(dt);
      force(2, motor_force_[2]);
      force(3, motor_force_[3]);
    } catch (const std::exception &ex) {
      zero_motors();
      RCLCPP_ERROR(nh_->get_logger(), "RISER write: %s", ex.what());
      return Ret::ERROR;
    }
    return Ret::OK;
  }
};
} // namespace riser_gazebo
PLUGINLIB_EXPORT_CLASS(riser_gazebo::AngularSystem,
                       gz_ros2_control::GazeboSimSystemInterface)
