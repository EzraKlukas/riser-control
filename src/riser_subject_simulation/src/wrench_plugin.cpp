#include "riser_subject_simulation/wrench_plugin.hpp"
#include <gz/sim/Link.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>

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

  // Transport callbacks run on another thread. Copy under the mutex, then
  // release it before touching physics; this is not a hard realtime path.
  Wrench left, right;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    left = left_wrench_;
    right = right_wrench_;
  }
  sim::Link(left_link_).AddWorldWrench(ecm, left.force_, left.torque_);
  sim::Link(right_link_).AddWorldWrench(ecm, right.force_, right.torque_);
}

static Wrench msg_to_wrench(const gz::msgs::EntityWrench &msg) {
  Wrench wrench_;
  wrench_.force_ = gz::math::Vector3d(msg.wrench().force().x(),
                                            msg.wrench().force().y(),
                                            msg.wrench().force().z());
  wrench_.torque_ = gz::math::Vector3d(msg.wrench().torque().x(),
                                             msg.wrench().torque().y(),
                                             msg.wrench().torque().z());
  return wrench_;
}

void WorldWrench::on_wrench(const gz::msgs::EntityWrench &msg) {
  const auto &entity = msg.entity();

  const bool correct_target = (entity.name() == "riser::left_force_plate" ||
                               entity.name() == "riser::right_force_plate") &&
                              entity.type() == gz::msgs::Entity::LINK;
  if (correct_target) {
    std::lock_guard<std::mutex> lock(command_mutex_);
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
    right_link_ = ecm.EntityByComponents(
        sim::components::Link(), sim::components::Name("right_force_plate"),
        sim::components::ParentEntity(model));
  }
}
} // namespace riser_subject_simulation

// Registers the class and its lifecycle interfaces so Gazebo can load it.
GZ_ADD_PLUGIN(riser_subject_simulation::WorldWrench,
                    gz::sim::System,
                    gz::sim::ISystemConfigure,
                    gz::sim::ISystemPreUpdate)
