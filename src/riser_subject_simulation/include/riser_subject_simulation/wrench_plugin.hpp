#pragma once

#include <math.h>
#include <memory>
#include <mutex>

#include <gz/sim/System.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/transport/Node.hh>

struct Wrench {
  gz::math::Vector3d force_;
  gz::math::Vector3d torque_;
};

namespace riser_subject_simulation {
namespace sim = gz::sim;

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
  void on_wrench(const gz::msgs::EntityWrench &msg);
  void resolve_target_link(sim::EntityComponentManager &ecm);

  sim::Entity left_link_ = sim::kNullEntity;
  sim::Entity right_link_ = sim::kNullEntity;

  gz::transport::Node node_;

  std::mutex command_mutex_;
  Wrench left_wrench_;
  Wrench right_wrench_;
};
} // namespace riser_subject_simulation
