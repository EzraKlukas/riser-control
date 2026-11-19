#include "riser_gazebo/physics_clock.hpp"
#include "riser_human_balance/ankle_balance.hpp"

#include <chrono>
#include <cmath>
#include <gz/msgs/double_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <limits>
#include <optional>

namespace riser_human_balance {
namespace sim = gz::sim;

class HumanBalanceSystem final : public sim::System,
                                 public sim::ISystemConfigure,
                                 public sim::ISystemPreUpdate {
public:
  void Configure(const sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 sim::EntityComponentManager &ecm,
                 sim::EventManager &) override {
    participant_ = sim::Model(entity);
    ankle_ = participant_.JointByName(ecm, "ankle");
    body_ = participant_.LinkByName(ecm, "body");
    feet_ = participant_.LinkByName(ecm, "feet");
    pulse_nm_ = sdf->Get<double>("pulse_nm", 0.).first;
    if (ankle_ == sim::kNullEntity || body_ == sim::kNullEntity ||
        feet_ == sim::kNullEntity || !std::isfinite(pulse_nm_) ||
        std::abs(pulse_nm_) > 50.) {
      gzerr << "Invalid RISER participant joints or pulse setting\n";
      return;
    }
    sim::Joint(ankle_).EnablePositionCheck(ecm);
    sim::Joint(ankle_).EnableVelocityCheck(ecm);
    ecm.CreateComponent(ankle_, sim::components::JointForceCmd({0.}));
    telemetry_ =
        transport_.Advertise<gz::msgs::Double_V>("/riser/human_balance");
    configured_ = true;
  }

  void PreUpdate(const sim::UpdateInfo &info,
                 sim::EntityComponentManager &ecm) override {
    if (!configured_ || info.paused || info.dt.count() <= 0)
      return;
    if (robot_ == sim::kNullEntity && !attach(ecm))
      return;

    auto &ankle_command =
        ecm.Component<sim::components::JointForceCmd>(ankle_)->Data()[0];
    const auto *clock =
        ecm.Component<riser_gazebo::PhysicsClockComponent>(robot_);
    const auto foot = read_pitch(ecm, foot_pitch_);
    const auto board = read_pitch(ecm, backboard_pitch_);
    const auto ankle = read_pitch(ecm, ankle_);
    if (!clock || !foot || !board || !ankle) {
      ankle_command = 0.;
      return;
    }
    const Pitch body{foot->angle_rad + ankle->angle_rad,
                     foot->rate_rad_s + ankle->rate_rad_s};
    const double now_s = std::chrono::duration<double>(info.simTime).count();
    if (!clock->Data().angular_velocity_active)
      held_before_release_ = true;
    const bool active =
        held_before_release_ && clock->Data().angular_velocity_active;
    if (!active) {
      release_time_s_.reset();
    } else if (!release_time_s_) {
      release_time_s_ = now_s;
    }

    // Joint effort applies a real action/reaction pair to the body and feet.
    // The virtual strap contributes only an equal/opposite pitch moment.
    ankle_command =
        active ? ankle_torque_nm(body, now_s - *release_time_s_, pulse_nm_)
               : 0.;
    const double strap = strap_torque_nm(body, *board);
    sim::Link(body_).AddWorldWrench(ecm, {}, {0., strap, 0.});
    sim::Link(backboard_).AddWorldWrench(ecm, {}, {0., -strap, 0.});

    if (now_s - last_publish_s_ >= 0.02 && telemetry_.HasConnections()) {
      gz::msgs::Double_V message;
      for (const double value :
           {now_s, body.angle_rad, board->angle_rad, foot->angle_rad,
            ankle_command, strap, active ? 1. : 0.})
        message.add_data(value);
      telemetry_.Publish(message);
      last_publish_s_ = now_s;
    }
  }

private:
  bool attach(sim::EntityComponentManager &ecm) {
    const auto robot = ecm.EntityByComponents(sim::components::Model(),
                                              sim::components::Name("riser"));
    if (robot == sim::kNullEntity)
      return false;
    const sim::Model model(robot);
    const auto plate = model.LinkByName(ecm, "left_force_plate");
    backboard_ = model.LinkByName(ecm, "tilt_frame");
    foot_pitch_ = model.JointByName(ecm, "foot_plate_pitch");
    backboard_pitch_ = model.JointByName(ecm, "backboard_pitch");
    if (plate == sim::kNullEntity || backboard_ == sim::kNullEntity ||
        foot_pitch_ == sim::kNullEntity || backboard_pitch_ == sim::kNullEntity)
      return false;

    // Both visible feet are a single rigid link. One weld places the full load
    // on the instrumented platform without a redundant closed kinematic loop.
    const auto weld = ecm.CreateEntity();
    ecm.CreateComponent(
        weld, sim::components::DetachableJoint({plate, feet_, "fixed"}));
    robot_ = robot;
    return true;
  }

  static std::optional<Pitch> read_pitch(const sim::EntityComponentManager &ecm,
                                         sim::Entity joint) {
    const auto *position = ecm.Component<sim::components::JointPosition>(joint);
    const auto *velocity = ecm.Component<sim::components::JointVelocity>(joint);
    if (!position || !velocity || position->Data().empty() ||
        velocity->Data().empty())
      return std::nullopt;
    const Pitch pitch{position->Data()[0], velocity->Data()[0]};
    if (!std::isfinite(pitch.angle_rad) || !std::isfinite(pitch.rate_rad_s))
      return std::nullopt;
    return pitch;
  }

  sim::Model participant_;
  sim::Entity ankle_ = sim::kNullEntity, feet_ = sim::kNullEntity,
              body_ = sim::kNullEntity, robot_ = sim::kNullEntity,
              foot_pitch_ = sim::kNullEntity,
              backboard_pitch_ = sim::kNullEntity,
              backboard_ = sim::kNullEntity;
  bool configured_ = false;
  bool held_before_release_ = false;
  double pulse_nm_ = 0.;
  std::optional<double> release_time_s_;
  double last_publish_s_ = -std::numeric_limits<double>::infinity();
  gz::transport::Node transport_;
  gz::transport::Node::Publisher telemetry_;
};
} // namespace riser_human_balance

GZ_ADD_PLUGIN(riser_human_balance::HumanBalanceSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
