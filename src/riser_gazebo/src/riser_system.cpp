// Humble + Fortress. Direct physical pitch torques with removed
// stage/rod dynamics reduced onto pitch.
// No physical pitch-joint position/velocity resets are used.
#include "riser_gazebo/reference_mapping.hpp"
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

constexpr std::size_t kPublicJointCount = PublicJointCount;
constexpr std::size_t kAngularInterfaceCount = AngularCount;
constexpr std::size_t kLinearInterfaceCount = ActuatorCount;

// Startup selection only. Runtime claim switching is intentionally not handled.
static CommandMode str_to_cmd_mode(const std::string &mode) {
  if (mode == "position" || mode == "angular_position")
    return CommandMode::AngularPosition;
  if (mode == "velocity" || mode == "angular_velocity")
    return CommandMode::AngularVelocity;
  if (mode == "actuator_position")
    return CommandMode::ActuatorPosition;
  if (mode == "actuator_velocity")
    return CommandMode::ActuatorVelocity;
  return CommandMode::None;
}

using ReturnType = hardware_interface::return_type;

// Loaded by gz_ros2_control through pluginlib, rather than directly by Gazebo.
// That wrapper reads state/updates controllers in PostUpdate and calls write in
// PreUpdate. This class implements hardware callbacks, not Gazebo update hooks.
class AngularSystem final : public gz_ros2_control::GazeboSimSystemInterface {
  sim::EntityComponentManager *ecm_ = nullptr;
  // Public angles use radians, actuator strokes use metres from neutral.
  // Only ML couples two physical joints.
  std::array<std::string, kPublicJointCount> public_joint_names_{
      {"backboard_pitch", "foot_plate_pitch", "ml_angle", "top_actuator",
       "torso_actuator", "hip_actuator", "bottom_actuator"}};
  // Physical indices: 0/1 are revolute pitch joints; 2/3 are prismatic slides.
  std::array<std::string, 4> physical_joint_names_{
      {"backboard_pitch", "foot_plate_pitch", "torso_lateral", "hip_lateral"}};
  std::array<sim::Entity, 4> physical_joints_{};
  // State/reference arrays use PublicJoint indices. Commands have separate
  // angular/actuator storage; physical reference arrays use kin::AxisIndex.
  std::array<double, kPublicJointCount> position_{}, velocity_{}, effort_{},
      reference_position_{}, reference_velocity_{};

  // 14 total command interfaces.
  std::array<double, kAngularInterfaceCount> angular_position_command_{};
  std::array<double, kAngularInterfaceCount> angular_velocity_command_{};
  std::array<double, kLinearInterfaceCount> actuator_position_command_{};
  std::array<double, kLinearInterfaceCount> actuator_velocity_command_{};
  std::array<double, kLinearInterfaceCount> limit_switch_lower_{},
      limit_switch_upper_{};
  double limit_switch_offset_ = 0.003; // metres inside each hard stop
  std::array<double, kPublicJointCount> position_lower_limits_{},
      position_upper_limits_{}, velocity_limits_{};
  std::array<double, 4>
      drive_effort_{}; // Pitch torques [N m], slide forces [N].
  std::array<double, 2> ml_heights_{};
  ReferenceMapping reference_mapping_;
  // Canonical mapping order: frame pitch, torso slide, hip slide, foot pitch.
  kin::Coordinates physical_position_{}, physical_velocity_{},
      measured_jacobian_{};
  kin::Coordinates physical_reference_position_{},
      physical_reference_velocity_{};
  kin::Coordinates neutral_cad_{};
  bool references_seeded_ = false;
  sensor_msgs::msg::JointState physical_msg_;
  int64_t last_state_publish_ns_ = -1;
  double physical_state_rate_hz_ = 100;
  static constexpr size_t physical_count_ = 4;
  double lateral_kp_ = 0, lateral_kd_ = 0, lateral_force_limit_ = 0;
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
      // Tables retain saved-CAD coordinates, while Gazebo joints now measure
      // from vertical/flat neutral. A constant offset does not change rates.
      const auto axis = axis_index == 0 ? kin::FramePitch : kin::FootPitch;
      const auto sample =
          interpolate_pitch_dynamics(axis_index, angle + neutral_cad_[axis]);
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
  CommandMode command_mode_ = CommandMode::None;
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

  void seed_references() {
    reference_position_ = position_;
    reference_velocity_.fill(0);
    physical_reference_position_ = physical_position_;
    physical_reference_velocity_.fill(0);
    std::copy_n(position_.begin(), AngularCount,
                angular_position_command_.begin());
    angular_velocity_command_.fill(0);
    for (std::size_t axis = 0; axis < ActuatorCount; ++axis)
      actuator_position_command_[axis] = position_[PublicForAxis[axis]];
    actuator_velocity_command_.fill(0);
    references_seeded_ = true;
  }

public:
  bool initSim(rclcpp::Node::SharedPtr &node, // is node the cm?
               std::map<std::string, sim::Entity> &gazebo_joints,
               const hardware_interface::HardwareInfo &hardware_info,
               sim::EntityComponentManager &ecm, int &) override {
    if (hardware_interface::SystemInterface::on_init(hardware_info) !=
        hardware_interface::CallbackReturn::SUCCESS)
      return false;
    nh_ = node;
    ecm_ = &ecm;
    try {
      command_mode_ =
          str_to_cmd_mode(hardware_info.hardware_parameters.at("command_mode"));
      if (command_mode_ == CommandMode::None)
        throw std::runtime_error("Unsupported command mode");
      // Resolve all four physical Gazebo joints, including the two slides
      // whose actuator resources are separately exported below.
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
      if (!std::isfinite(lateral_kp_) || lateral_kp_ <= 0 ||
          !std::isfinite(lateral_kd_) || lateral_kd_ < 0 ||
          !std::isfinite(lateral_force_limit_) || lateral_force_limit_ <= 0)
        throw std::runtime_error("Invalid lateral servo gains");
      // Angular and actuator limits come from Xacro, in rad and m respectively.
      const auto switch_offset = hardware_info.hardware_parameters.find("limit_switch_offset");
      if (switch_offset != hardware_info.hardware_parameters.end())
        limit_switch_offset_ = std::stod(switch_offset->second);
      if (hardware_info.joints.size() != kPublicJointCount)
        throw std::runtime_error("Exactly seven public joints required");
      for (size_t axis_index = 0; axis_index < kPublicJointCount;
           ++axis_index) {
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
      Limits limits{};
      for (const auto joint : PublicForAxis)
        if (!std::isfinite(limit_switch_offset_) || limit_switch_offset_ < 0 ||
            2 * limit_switch_offset_ >=
                position_upper_limits_[joint] - position_lower_limits_[joint])
          throw std::runtime_error("Invalid limit-switch offset");
      for (std::size_t i = 0; i < PublicJointCount; ++i)
        limits[i] = {position_lower_limits_[i], position_upper_limits_[i],
                     velocity_limits_[i]};
      std::istringstream neutral_stream(
          hardware_info.hardware_parameters.at("neutral_cad"));
      for (auto &offset : neutral_cad_)
        if (!(neutral_stream >> offset))
          throw std::runtime_error("Malformed CAD neutral offsets");
      neutral_stream >> std::ws;
      if (!neutral_stream.eof())
        throw std::runtime_error("Extra CAD neutral offsets");
      if (reference_mapping_.configure(limits, ml_heights_, neutral_cad_) !=
          kin::Error::None)
        throw std::runtime_error("Invalid actuator/ML mapping configuration");
      for (std::size_t i = 0; i < 2; ++i) {
        const auto axis = i == 0 ? kin::FramePitch : kin::FootPitch;
        const auto &table = reduced_pitch_tables_[i];
        const auto joint = i == 0 ? Backboard : FootPlate;
        if (limits[joint].lower + neutral_cad_[axis] < table.front().angle ||
            limits[joint].upper + neutral_cad_[axis] > table.back().angle)
          RCLCPP_WARN(
              node->get_logger(),
              "%s pitch travel exceeds CAD dynamics table [%g, %g] rad; "
              "using endpoint coefficients outside it until CAD resampling",
              i == 0 ? "Top" : "Bottom", table.front().angle,
              table.back().angle);
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
    for (size_t joint_index = 0; joint_index < kPublicJointCount;
         ++joint_index) {
      interfaces.emplace_back(public_joint_names_[joint_index], "position",
                              &position_[joint_index]);
      interfaces.emplace_back(public_joint_names_[joint_index], "velocity",
                              &velocity_[joint_index]);
      interfaces.emplace_back(public_joint_names_[joint_index], "effort",
                              &effort_[joint_index]);
    }
    for (std::size_t axis = 0; axis < ActuatorCount; ++axis) {
      const auto &name = public_joint_names_[PublicForAxis[axis]];
      interfaces.emplace_back(name, "limit_switch_lower",
                              &limit_switch_lower_[axis]);
      interfaces.emplace_back(name, "limit_switch_upper",
                              &limit_switch_upper_[axis]);
    }
    return interfaces;
  }
  // Both types are registered; write() consumes only the startup-selected
  // family. Claim-based exclusivity/runtime mode switching remains future work.
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (size_t i = 0; i < kAngularInterfaceCount; ++i) {
      interfaces.emplace_back(public_joint_names_[i], "position",
                              &angular_position_command_[i]);
      interfaces.emplace_back(public_joint_names_[i], "velocity",
                              &angular_velocity_command_[i]);
    }
    for (size_t axis = 0; axis < ActuatorCount; ++axis) {
      interfaces.emplace_back(public_joint_names_[PublicForAxis[axis]],
                              "position", &actuator_position_command_[axis]);
      interfaces.emplace_back(public_joint_names_[PublicForAxis[axis]],
                              "velocity", &actuator_velocity_command_[axis]);
    }
    return interfaces;
  }
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override {
    active_ = true;
    references_seeded_ = false;
    if (state_ready_)
      seed_references();
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
      kin::Coordinates q{}, qdot{};
      for (std::size_t axis = 0; axis < ActuatorCount; ++axis) {
        q[axis] = joint_state(PhysicalForAxis[axis]);
        qdot[axis] = joint_state(PhysicalForAxis[axis], true);
      }
      // observe() fills all seven states, including s=f(q), sdot=J*qdot for
      // the two virtual pitch actuators and the signed lateral strokes.
      if (reference_mapping_.observe(q, qdot, position_, velocity_,
                                     measured_jacobian_) != kin::Error::None)
        throw std::runtime_error("Invalid measured actuator/ML state");
      // Publish switches only in read(): the wrapper then updates controllers
      // before the next write can enforce them. Include contact overshoot and
      // mapping roundoff rather than requiring exact floating-point equality.
      constexpr double switch_tolerance = 1e-10; // metres
      for (std::size_t axis = 0; axis < ActuatorCount; ++axis) {
        const auto joint = PublicForAxis[axis];
        limit_switch_lower_[axis] =
            position_[joint] <= position_lower_limits_[joint] +
                                    limit_switch_offset_ + switch_tolerance;
        limit_switch_upper_[axis] =
            position_[joint] >= position_upper_limits_[joint] -
                                    limit_switch_offset_ - switch_tolerance;
      }
      physical_position_ = q;
      physical_velocity_ = qdot;
      effort_[Backboard] = drive_effort_[BackboardJoint];
      effort_[FootPlate] = drive_effort_[FootPlateJoint];
      const double tangent = std::tan(position_[Ml]);
      effort_[Ml] = (1 + tangent * tangent) *
                    (ml_heights_[0] * drive_effort_[TorsoJoint] +
                     ml_heights_[1] * drive_effort_[HipJoint]);
      // Virtual work: tau_q = J * F_actuator. These are commanded equivalent
      // efforts, NOT measured motor forces or reconstructed linkage reactions.
      for (std::size_t axis = 0; axis < ActuatorCount; ++axis)
        effort_[PublicForAxis[axis]] =
            drive_effort_[PhysicalForAxis[axis]] / measured_jacobian_[axis];
      if (last_state_publish_ns_ < 0 ||
          now.nanoseconds() - last_state_publish_ns_ >=
              1e9 / physical_state_rate_hz_) {
        physical_msg_.header.stamp = now;
        for (size_t axis_index = 0; axis_index < physical_count_;
             ++axis_index) {
          physical_msg_.position[axis_index] = joint_state(axis_index);
          physical_msg_.velocity[axis_index] = joint_state(axis_index, true);
        }
        physical_msg_.position[physical_count_] = position_[Ml];
        physical_msg_.velocity[physical_count_] = velocity_[Ml];
        state_pub_->publish(physical_msg_);
        last_state_publish_ns_ = now.nanoseconds();
      }
      state_ready_ = true;
    } catch (const std::exception &ex) {
      if (state_ready_) {
        // Empty components are tolerated only until the first successful
        // read.
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
    if (last_write_ns_ >= 0 && write_time_ns < last_write_ns_) {
      previous_pitch_velocity_valid_ = false;
      references_seeded_ = false;
    }
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
          apply_pitch_efforts(step_seconds); // applies joint efforts.
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
      if (!references_seeded_)
        seed_references();
      PublicCoordinates commands{}; // one unified command collector.
      if (angular(command_mode_)) {
        const auto &source = velocity(command_mode_)
                                 ? angular_velocity_command_
                                 : angular_position_command_;
        std::copy(source.begin(), source.end(), commands.begin());
      } else {
        const auto &source = velocity(command_mode_)
                                 ? actuator_velocity_command_
                                 : actuator_position_command_;
        for (std::size_t axis = 0; axis < ActuatorCount; ++axis)
          commands[PublicForAxis[axis]] = source[axis];
      }
      // Steps 0 / 0.5 / 1 live in ReferenceMapping::update:
      // validate/integrate native commands once, convert with f^-1 and J, and
      // recheck both coordinate systems. Failure leaves references unchanged.
      auto next_position = reference_position_;
      auto next_velocity = reference_velocity_;
      auto next_physical = physical_reference_position_;
      auto next_physical_velocity = physical_reference_velocity_;
      if (reference_mapping_.update(
              command_mode_, commands, step_seconds, next_position,
              next_velocity, next_physical, next_physical_velocity) !=
          kin::Error::None)
        throw std::domain_error("Invalid or out-of-bounds mapped reference");

      for (std::size_t axis = 0; axis < ActuatorCount; ++axis) {
        const auto joint = PublicForAxis[axis];
        const double advance = next_position[joint] - reference_position_[joint];
        constexpr double tolerance = 1e-10; // metres; mapping roundoff only
        if ((limit_switch_lower_[axis] && advance < -tolerance) ||
            (limit_switch_upper_[axis] && advance > tolerance))
          throw std::domain_error(public_joint_names_[joint] +
                                  " reference advances into active limit switch");
      }
      reference_position_ = next_position;
      reference_velocity_ = next_velocity;
      physical_reference_position_ = next_physical;
      physical_reference_velocity_ = next_physical_velocity;

      // 2. Pitch servos consume independent angular targets. Actuator modes
      // supplied these through the nonlinear inverse; reduced physics is
      // intact.
      for (const auto axis : {kin::FramePitch, kin::FootPitch}) {
        const auto joint = PhysicalForAxis[axis];
        drive_effort_[joint] =
            pitch_kp_[joint] *
                (physical_reference_position_[axis] - joint_state(joint)) +
            pitch_kd_[joint] *
                (physical_reference_velocity_[axis] - joint_state(joint, true));
      }
      // 3. Slide servos always consume PHYSICAL slide coordinates, not strokes.
      // Angular mode expands ML; actuator mode preserves each independent slide
      // and the canonical stroke sign (currently both lateral ratios are -1).
      for (const auto axis : {kin::TorsoLateral, kin::HipLateral}) {
        const auto joint = PhysicalForAxis[axis];
        drive_effort_[joint] =
            lateral_kp_ *
                (physical_reference_position_[axis] - joint_state(joint)) +
            lateral_kd_ *
                (physical_reference_velocity_[axis] - joint_state(joint, true));
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
