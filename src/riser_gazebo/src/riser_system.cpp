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
using ReturnType = hardware_interface::return_type;

// Loaded by gz_ros2_control through pluginlib, rather than directly by Gazebo.
// That wrapper reads state/updates controllers in PostUpdate and calls write in
// PreUpdate. This class implements hardware callbacks, not Gazebo update hooks.
class AngularSystem final : public gz_ros2_control::GazeboSimSystemInterface {
  sim::EntityComponentManager *ecm_ = nullptr;
  // Public interfaces use radians. Only ML couples two physical joints.
  std::array<std::string, 3> public_joint_names_{
      {"backboard_pitch", "foot_plate_pitch", "ml_angle"}};
  // Physical indices: 0/1 are revolute pitch joints; 2/3 are prismatic slides.
  std::array<std::string, 4> physical_joint_names_{
      {"backboard_pitch", "foot_plate_pitch", "torso_lateral", "hip_lateral"}};
  std::array<sim::Entity, 4> physical_joints_{};
  // Exported interfaces hold pointers into these arrays. Controllers write
  // command_; write() converts it into references and then physical efforts.
  std::array<double, 3> position_{}, velocity_{}, effort_{}, command_{},
      reference_position_{}, reference_velocity_{};
  std::array<double, 3> position_lower_limits_{}, position_upper_limits_{},
      velocity_limits_{};
  std::array<double, 4>
      drive_effort_{}; // Pitch torques [N m], slide forces [N].
  std::array<double, 2> ml_heights_{};
  sensor_msgs::msg::JointState physical_msg_;
  int64_t last_state_publish_ns_ = -1;
  double physical_state_rate_hz_ = 100;
  static constexpr size_t physical_count_ = 4;
  double lateral_kp_ = 0, lateral_kd_ = 0, lateral_force_limit_ = 0;
  // Table columns: angle, inertia, inertia derivative, gravity torque, damping.
  struct ReducedSample {
    double angle, inertia, inertia_derivative, gravity_torque, damping;
  };
  std::array<std::vector<ReducedSample>, 2> reduced_pitch_tables_;
  std::array<double, 2> pitch_kp_{}, pitch_kd_{}, pitch_torque_limits_{},
      previous_pitch_velocity_{};
  bool previous_pitch_velocity_valid_ = false;

  // Tables are validated as uniformly spaced during initSim. Clamp to the
  // table domain and linearly interpolate each reduced-dynamics coefficient.
  ReducedSample interpolate_pitch_dynamics(size_t axis_index,
                                           double angle) const {
    const auto &table = reduced_pitch_tables_[axis_index];
    const double step = table[1].angle - table[0].angle;
    const double index = std::clamp((angle - table.front().angle) / step, 0.,
                                    double(table.size() - 1));
    const size_t sample_index = std::min(size_t(index), table.size() - 2);
    const double fraction = index - sample_index;
    const auto &left = table[sample_index];
    const auto &right = table[sample_index + 1];
    return {angle, left.inertia + fraction * (right.inertia - left.inertia),
            left.inertia_derivative +
                fraction * (right.inertia_derivative - left.inertia_derivative),
            left.gravity_torque +
                fraction * (right.gravity_torque - left.gravity_torque),
            left.damping + fraction * (right.damping - left.damping)};
  }
  // Add the effects of the pitch stages/rods omitted from the Gazebo model.
  // The actuator servo effort was already clamped; these passive terms are
  // added afterward, including while the hardware component is inactive.
  void apply_pitch_efforts(double step_seconds) {
    for (size_t axis_index = 0; axis_index < 2; ++axis_index) {
      const double angle = joint_state(axis_index),
                   angular_velocity = joint_state(axis_index, true);
      if (!std::isfinite(angle) || !std::isfinite(angular_velocity))
        throw std::domain_error("Nonfinite direct pitch state");
      const double acceleration =
          previous_pitch_velocity_valid_
              ? (angular_velocity - previous_pitch_velocity_[axis_index]) /
                    step_seconds
              : 0.;
      const auto sample = interpolate_pitch_dynamics(axis_index, angle);
      // M(q) qdd + 1/2 M'(q) qd^2 + D(q) qd - Qg(q): the
      // complete planar contribution of the removed audited stage and rod.
      // Acceleration feedback is delayed by one physics step. This numerical
      // approximation is quantified in the historical runtime validation.
      set_joint_effort(axis_index, drive_effort_[axis_index] -
                                       sample.inertia * acceleration -
                                       .5 * sample.inertia_derivative *
                                           angular_velocity * angular_velocity -
                                       sample.damping * angular_velocity +
                                       sample.gravity_torque);
      previous_pitch_velocity_[axis_index] = angular_velocity;
    }
    previous_pitch_velocity_valid_ = true;
  }
  bool state_ready_ = false, active_ = false;
  int64_t last_write_ns_ = -1;
  std::string command_mode_ = "position";
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  double
  hardware_parameter(const hardware_interface::HardwareInfo &hardware_info,
                     const std::string &parameter_name) {
    return std::stod(hardware_info.hardware_parameters.at(parameter_name));
  }
  // Read Gazebo's first DOF; all four physical joints have one DOF.
  double joint_state(size_t axis_index, bool read_velocity = false) const {
    if (read_velocity) {
      auto component = ecm_->Component<sim::components::JointVelocity>(
          physical_joints_[axis_index]);
      if (!component || component->Data().empty())
        throw std::runtime_error("Missing joint velocity");
      return component->Data()[0];
    }
    auto component = ecm_->Component<sim::components::JointPosition>(
        physical_joints_[axis_index]);
    if (!component || component->Data().empty())
      throw std::runtime_error("Missing joint position");
    return component->Data()[0];
  }
  // JointForceCmd means torque for a revolute joint, force for a prismatic one.
  // Physics integrates this effort; no position or velocity is reset here.
  void set_joint_effort(size_t joint, double effort) {
    auto component = ecm_->Component<sim::components::JointForceCmd>(
        physical_joints_[joint]);
    if (component)
      component->Data() = {effort};
    else
      ecm_->CreateComponent(physical_joints_[joint],
                            sim::components::JointForceCmd({effort}));
  }
  void zero_drive_efforts() {
    for (size_t axis_index = 0; axis_index < physical_count_; ++axis_index)
      set_joint_effort(axis_index, 0);
    drive_effort_.fill(0);
  }

public:
  bool initSim(rclcpp::Node::SharedPtr &node,
               std::map<std::string, sim::Entity> &gazebo_joints,
               const hardware_interface::HardwareInfo &hardware_info,
               sim::EntityComponentManager &ecm, int &) override {
    if (hardware_interface::SystemInterface::on_init(hardware_info) !=
        hardware_interface::CallbackReturn::SUCCESS)
      return false;
    nh_ = node;
    ecm_ = &ecm;
    try {
      command_mode_ = hardware_info.hardware_parameters.at("command_mode");
      if (command_mode_ != "position" && command_mode_ != "velocity")
        throw std::runtime_error("Unsupported command mode");
      // Resolve all four physical Gazebo joints, including the two slides
      // that are not exported as individually commandable public joints.
      for (size_t axis_index = 0; axis_index < physical_count_; ++axis_index) {
        physical_joints_[axis_index] =
            gazebo_joints.at(physical_joint_names_[axis_index]);
        if (!ecm.Component<sim::components::JointPosition>(
                physical_joints_[axis_index]))
          ecm.CreateComponent(physical_joints_[axis_index],
                              sim::components::JointPosition());
        if (!ecm.Component<sim::components::JointVelocity>(
                physical_joints_[axis_index]))
          ecm.CreateComponent(physical_joints_[axis_index],
                              sim::components::JointVelocity());
      }
      // Pitch stages are represented by dynamics tables, not linear joints.
      for (size_t axis_index = 0; axis_index < 2; ++axis_index) {
        const std::string prefix = axis_index == 0 ? "top_" : "bottom_";
        std::istringstream table_stream(
            hardware_info.hardware_parameters.at(prefix + "reduced_table"));
        ReducedSample sample;
        while (table_stream >> sample.angle >> sample.inertia >>
               sample.inertia_derivative >> sample.gravity_torque >>
               sample.damping) {
          if (!std::isfinite(sample.angle) || !std::isfinite(sample.inertia) ||
              !std::isfinite(sample.inertia_derivative) ||
              !std::isfinite(sample.gravity_torque) ||
              !std::isfinite(sample.damping) || sample.inertia < 0 ||
              sample.damping < 0)
            throw std::runtime_error("Invalid reduced dynamics");
          reduced_pitch_tables_[axis_index].push_back(sample);
        }
        const auto &table = reduced_pitch_tables_[axis_index];
        if (!table_stream.eof() || table.size() < 2)
          throw std::runtime_error("Malformed reduced dynamics table");
        const double step = table[1].angle - table[0].angle;
        if (step <= 0)
          throw std::runtime_error("Invalid reduced dynamics grid");
        for (size_t sample_index = 1; sample_index < table.size();
             ++sample_index)
          if (std::abs(table[sample_index].angle -
                       table[sample_index - 1].angle - step) > 1e-10)
            throw std::runtime_error("Nonuniform reduced dynamics grid");
        pitch_kp_[axis_index] =
            hardware_parameter(hardware_info, prefix + "pitch_kp");
        pitch_kd_[axis_index] =
            hardware_parameter(hardware_info, prefix + "pitch_kd");
        pitch_torque_limits_[axis_index] =
            hardware_parameter(hardware_info, prefix + "pitch_max_torque");
        if (!std::isfinite(pitch_kp_[axis_index]) ||
            !std::isfinite(pitch_kd_[axis_index]) ||
            !std::isfinite(pitch_torque_limits_[axis_index]) ||
            pitch_kp_[axis_index] <= 0 || pitch_kd_[axis_index] < 0 ||
            pitch_torque_limits_[axis_index] <= 0)
          throw std::runtime_error("Invalid angular servo gains");
      }
      ml_heights_ = {hardware_parameter(hardware_info, "torso_height"),
                     hardware_parameter(hardware_info, "hip_height")};
      lateral_kp_ = hardware_parameter(hardware_info, "servo_kp");
      lateral_kd_ = hardware_parameter(hardware_info, "servo_kd");
      lateral_force_limit_ =
          hardware_parameter(hardware_info, "servo_max_force");
      // Public angular limits and slide servo settings come from the Xacro.
      if (hardware_info.joints.size() != 3)
        throw std::runtime_error("Exactly three public joints required");
      for (size_t axis_index = 0; axis_index < 3; ++axis_index) {
        auto joint =
            std::find_if(hardware_info.joints.begin(),
                         hardware_info.joints.end(), [&](const auto &joint) {
                           return joint.name == public_joint_names_[axis_index];
                         });
        if (joint == hardware_info.joints.end())
          throw std::runtime_error("Missing public joint");
        position_lower_limits_[axis_index] =
            std::stod(joint->parameters.at("lower"));
        position_upper_limits_[axis_index] =
            std::stod(joint->parameters.at("upper"));
        velocity_limits_[axis_index] =
            std::stod(joint->parameters.at("velocity_limit"));
      }
      physical_state_rate_hz_ =
          hardware_parameter(hardware_info, "physical_state_rate");
      if (!std::isfinite(physical_state_rate_hz_) ||
          physical_state_rate_hz_ <= 0)
        throw std::runtime_error("Invalid physical state rate");
      // Publish physical joint states separately for TF/inspection. Publishing
      // a state here does not export a ros2_control resource interface for it.
      physical_msg_.name.assign(physical_joint_names_.begin(),
                                physical_joint_names_.begin() +
                                    physical_count_);
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
  // Each handle exposes the storage itself, not a copy of its current value.
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (size_t axis_index = 0; axis_index < 3; ++axis_index) {
      interfaces.emplace_back(public_joint_names_[axis_index], "position",
                              &position_[axis_index]);
      interfaces.emplace_back(public_joint_names_[axis_index], "velocity",
                              &velocity_[axis_index]);
      interfaces.emplace_back(public_joint_names_[axis_index], "effort",
                              &effort_[axis_index]);
    }
    return interfaces;
  }
  // Exactly one command type per public joint, selected at initialization.
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (size_t axis_index = 0; axis_index < 3; ++axis_index)
      interfaces.emplace_back(public_joint_names_[axis_index], command_mode_,
                              &command_[axis_index]);
    return interfaces;
  }
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override {
    // Hardware activation seeds position references from the last read.
    active_ = true;
    reference_position_ = position_;
    reference_velocity_.fill(0);
    command_ =
        command_mode_ == "position" ? position_ : std::array<double, 3>{};
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override {
    active_ = false;
    zero_drive_efforts();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  ReturnType read(const rclcpp::Time &now, const rclcpp::Duration &) override {
    try {
      position_[0] = joint_state(0);
      position_[1] = joint_state(1);
      velocity_[0] = joint_state(0, true);
      velocity_[1] = joint_state(1, true);
      // Least-squares fit of y_i = h_i * tan(ML). This still produces an angle
      // when the slides disagree; it does not detect or correct that mismatch.
      double torso_height = ml_heights_[0], hip_height = ml_heights_[1],
             height_squared_sum =
                 torso_height * torso_height + hip_height * hip_height,
             ml_tangent =
                 (torso_height * joint_state(2) + hip_height * joint_state(3)) /
                 height_squared_sum;
      position_[2] = std::atan(ml_tangent);
      velocity_[2] = (torso_height * joint_state(2, true) +
                      hip_height * joint_state(3, true)) /
                     (height_squared_sum * (1 + ml_tangent * ml_tangent));
      // Report the previous commanded drive efforts, not measured joint loads.
      // ML torque follows virtual work: tau = sum(h_i * sec(ML)^2 * F_i).
      effort_[0] = drive_effort_[0];
      effort_[1] = drive_effort_[1];
      effort_[2] =
          (1 + ml_tangent * ml_tangent) *
          (torso_height * drive_effort_[2] + hip_height * drive_effort_[3]);
      if (last_state_publish_ns_ < 0 ||
          now.nanoseconds() - last_state_publish_ns_ >=
              1e9 / physical_state_rate_hz_) {
        physical_msg_.header.stamp = now;
        for (size_t axis_index = 0; axis_index < physical_count_;
             ++axis_index) {
          physical_msg_.position[axis_index] = joint_state(axis_index);
          physical_msg_.velocity[axis_index] = joint_state(axis_index, true);
        }
        physical_msg_.position[physical_count_] = position_[2];
        physical_msg_.velocity[physical_count_] = velocity_[2];
        state_pub_->publish(physical_msg_);
        last_state_publish_ns_ = now.nanoseconds();
      }
      state_ready_ = true;
    } catch (const std::exception &ex) {
      if (state_ready_) {
        // Empty components are tolerated only until the first successful read.
        RCLCPP_ERROR(nh_->get_logger(), "RISER read: %s", ex.what());
        zero_drive_efforts();
        return ReturnType::ERROR;
      }
      return ReturnType::OK;
    }
    return ReturnType::OK;
  }
  ReturnType write(const rclcpp::Time &now, const rclcpp::Duration &) override {
    // gz_ros2_control passes time since its last controller update, not since
    // its previous physics-step write. Integrate only the actual write delta.
    const auto write_time_ns = now.nanoseconds();
    if (last_write_ns_ >= 0 && write_time_ns < last_write_ns_)
      previous_pitch_velocity_valid_ = false;
    double step_seconds =
        last_write_ns_ < 0
            ? 0.
            : std::max(0., (write_time_ns - last_write_ns_) * 1e-9);
    last_write_ns_ = write_time_ns;
    if (!state_ready_) {
      zero_drive_efforts();
      return ReturnType::OK;
    }
    if (!active_) {
      // Disable servo effort but retain the removed bodies' passive dynamics.
      zero_drive_efforts();
      try {
        if (step_seconds > 0)
          apply_pitch_efforts(step_seconds);
      } catch (const std::exception &ex) {
        RCLCPP_ERROR(nh_->get_logger(), "RISER passive dynamics: %s",
                     ex.what());
        return ReturnType::ERROR;
      }
      return ReturnType::OK;
    }
    try {
      if (step_seconds <= 0)
        return ReturnType::OK;
      // 1. Position commands are absolute targets. Velocity commands integrate
      // bounded references; reaching an angular limit zeros reference velocity.
      for (size_t axis_index = 0; axis_index < 3; ++axis_index) {
        if (!std::isfinite(command_[axis_index]))
          throw std::domain_error("Nonfinite command");
        if (command_mode_ == "position") {
          if (command_[axis_index] < position_lower_limits_[axis_index] ||
              command_[axis_index] > position_upper_limits_[axis_index])
            throw std::domain_error("Angular target outside bounds");
          reference_position_[axis_index] = command_[axis_index];
          reference_velocity_[axis_index] = 0;
        } else {
          reference_velocity_[axis_index] =
              std::clamp(command_[axis_index], -velocity_limits_[axis_index],
                         velocity_limits_[axis_index]);
          reference_position_[axis_index] =
              std::clamp(reference_position_[axis_index] +
                             step_seconds * reference_velocity_[axis_index],
                         position_lower_limits_[axis_index],
                         position_upper_limits_[axis_index]);
          if (reference_position_[axis_index] ==
                  position_lower_limits_[axis_index] ||
              reference_position_[axis_index] ==
                  position_upper_limits_[axis_index])
            reference_velocity_[axis_index] = 0;
        }
      }
      // 2. Pitch PD servos act directly on the two revolute Gazebo joints.
      for (size_t axis_index = 0; axis_index < 2; ++axis_index)
        drive_effort_[axis_index] =
            pitch_kp_[axis_index] *
                (reference_position_[axis_index] - joint_state(axis_index)) +
            pitch_kd_[axis_index] * (reference_velocity_[axis_index] -
                                     joint_state(axis_index, true));
      // 3. The single ML reference produces separate slide position/velocity
      // targets: y_i = h_i * tan(ML), ydot_i = h_i * sec(ML)^2 * MLdot.
      double ml_tangent = std::tan(reference_position_[2]);
      for (size_t axis_index = 0; axis_index < 2; ++axis_index) {
        drive_effort_[axis_index + 2] =
            lateral_kp_ * (ml_heights_[axis_index] * ml_tangent -
                           joint_state(axis_index + 2)) +
            lateral_kd_ *
                (ml_heights_[axis_index] * (1 + ml_tangent * ml_tangent) *
                     reference_velocity_[2] -
                 joint_state(axis_index + 2, true));
      }
      // 4. Limit actuator effort, add reduced pitch dynamics, then write all
      // four physical commands to the entity-component manager for physics.
      for (size_t axis_index = 0; axis_index < 4; ++axis_index) {
        const double limit = axis_index < 2 ? pitch_torque_limits_[axis_index]
                                            : lateral_force_limit_;
        drive_effort_[axis_index] =
            std::clamp(drive_effort_[axis_index], -limit, limit);
      }
      apply_pitch_efforts(step_seconds);
      set_joint_effort(2, drive_effort_[2]);
      set_joint_effort(3, drive_effort_[3]);
    } catch (const std::exception &ex) {
      zero_drive_efforts();
      RCLCPP_ERROR(nh_->get_logger(), "RISER write: %s", ex.what());
      return ReturnType::ERROR;
    }
    return ReturnType::OK;
  }
};
} // namespace riser_gazebo
PLUGINLIB_EXPORT_CLASS(riser_gazebo::AngularSystem,
                       gz_ros2_control::GazeboSimSystemInterface)
