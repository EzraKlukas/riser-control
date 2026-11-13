#include "riser_gazebo/physics_clock.hpp"
#include <chrono>
#include <gz/plugin/Register.hh>
#include <gz_ros2_control/gz_ros2_control_plugin.hpp>

namespace riser_gazebo {
// Preserve the binary plugin's lifecycle and read/update/write ordering. Only
// the synchronous physics context is added; /clock remains available to ROS.
class PhysicsClockControlPlugin
    : public gz_ros2_control::GazeboSimROS2ControlPlugin {
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &events) override {
    entity_ = entity;
    if (auto *clock = ecm.Component<PhysicsClockComponent>(entity_))
      clock->Data() = {};
    else
      ecm.CreateComponent(entity_, PhysicsClockComponent{});
    GazeboSimROS2ControlPlugin::Configure(entity, sdf, ecm, events);
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    auto &clock = ecm.Component<PhysicsClockComponent>(entity_)->Data();
    clock.time_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime)
            .count();
    clock.step_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count();
    clock.paused = info.paused;
    if (!info.paused)
      GazeboSimROS2ControlPlugin::PreUpdate(info, ecm);
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (!info.paused)
      GazeboSimROS2ControlPlugin::PostUpdate(info, ecm);
  }

private:
  gz::sim::Entity entity_ = gz::sim::kNullEntity;
};
} // namespace riser_gazebo

GZ_ADD_PLUGIN(riser_gazebo::PhysicsClockControlPlugin, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate)
