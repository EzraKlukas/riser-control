#pragma once

#include <math.h>
#include <memory>
#include <mutex>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/entity_wrench.pb.h>
#include <ignition/transport/Node.hh>

struct Wrench {
  ignition::math::Vector3d force_;
  ignition::math::Vector3d torque_;
};

namespace riser_subject_simulation {
namespace sim = ignition::gazebo;

class WorldWrench : public sim::System,
                    public sim::ISystemConfigure,
                    public sim::ISystemPreUpdate {
public:
  void Configure(const sim::Entity &,
                 const std::shared_ptr<const sdf::Element> &,
                 sim::EntityComponentManager &, sim::EventManager &) override;

  void PreUpdate(const sim::UpdateInfo &info,
                 sim::EntityComponentManager &ecm) override;

private:
  void on_wrench(const ignition::msgs::EntityWrench &msg);
  void resolve_target_link(sim::EntityComponentManager &ecm);

  sim::Entity left_link_ = sim::kNullEntity;
  sim::Entity right_link_ = sim::kNullEntity;

  ignition::transport::Node node_;

  std::mutex command_mutex_;
  Wrench left_wrench_;
  Wrench right_wrench_;
};
} // namespace riser_subject_simulation
