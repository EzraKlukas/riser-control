#include "riser_subject_simulation/wrench_plugin.hpp"
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/msgs/double.pb.h>
#include <ignition/plugin/Register.hh>

namespace riser_subject_simulation {
void WorldWrench::Configure(const sim::Entity &,
                            const std::shared_ptr<const sdf::Element> &,
                            sim::EntityComponentManager &,
                            sim::EventManager &) {
  node_.Subscribe("/riser/wrench_input", &WorldWrench::on_wrench, this);
}

void WorldWrench::PreUpdate(const sim::UpdateInfo &info,
                            sim::EntityComponentManager &ecm) {
  if (info.paused) {
    return;
  }

  // The model is spawned after the world plugin, so retry until it exists.
  if (left_link_ == sim::kNullEntity || right_link_ == sim::kNullEntity) {
    resolve_target_link(ecm);
  }
  if (left_link_ == sim::kNullEntity || right_link_ == sim::kNullEntity) {
    return;
  }

  sim::Link(left_link_)
      .AddWorldWrench(ecm, left_wrench_.force_, left_wrench_.torque_);
}

static Wrench msg_to_wrench(const ignition::msgs::EntityWrench &msg) {
  Wrench wrench_;
  wrench_.force_ = ignition::math::Vector3d(msg.wrench().force().x(),
                                            msg.wrench().force().y(),
                                            msg.wrench().force().z());
  wrench_.torque_ = ignition::math::Vector3d(msg.wrench().force().x(),
                                             msg.wrench().force().y(),
                                             msg.wrench().force().z());
  return wrench_;
}

void WorldWrench::on_wrench(const ignition::msgs::EntityWrench &msg) {
  const auto &entity = msg.entity();

  const bool correct_target = (entity.name() == "riser::left_force_plate" ||
                               entity.name() == "riser::right_force_plate") &&
                              entity.type() == ignition::msgs::Entity::LINK;
  if (correct_target) {
    if (entity.name() == "riser::left_force_plate") {
      left_wrench_ = msg_to_wrench(msg);
    } else {
      right_wrench_ = msg_to_wrench(msg);
    }
  }
}
void WorldWrench::resolve_target_link(sim::EntityComponentManager &ecm) {
  const auto model = ecm.EntityByComponents(sim::components::Model(),
                                            sim::components::Name("riser"));

  if (model != sim::kNullEntity) {
    left_link_ = ecm.EntityByComponents(
        sim::components::Link(), sim::components::Name("left_force_plate"),
        sim::components::ParentEntity(model));
    left_link_ = ecm.EntityByComponents(
        sim::components::Link(), sim::components::Name("right_force_plate"),
        sim::components::ParentEntity(model));
  }
}
} // namespace riser_subject_simulation

// Registers the class and its lifecycle interfaces so Gazebo can load it.
IGNITION_ADD_PLUGIN(riser_subject_simulation::WorldWrench,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)
