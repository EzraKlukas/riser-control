// Simulation-only prescribed wrench fixture, never part of a hardware backend.
// Loads are applied to real plate bodies; they are not injected into sensor
// data.
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Element.hh>
#include <vector>

namespace riser_gazebo {
namespace sim = ignition::gazebo;
class LoadingFixture final : public sim::System,
                             public sim::ISystemConfigure,
                             public sim::ISystemPreUpdate {
  struct Load {
    std::string side;
    ignition::math::Vector3d force, offset, torque;
    double start, duration;
    sim::Entity link = sim::kNullEntity;
  };
  std::vector<Load> loads_;

public:
  void Configure(const sim::Entity &,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 sim::EntityComponentManager &, sim::EventManager &) override {
    if (!sdf->HasElement("load"))
      return;
    // Each SDF <load> specifies a link-local wrench, application offset and
    // simulation-time interval. Resolve the actual plate entity after spawning.
    auto load_element = sdf->FindElement("load");
    while (load_element) {
      Load load;
      load.side = load_element->Get<std::string>("side");
      if (load.side != "left" && load.side != "right")
        throw std::runtime_error("Invalid fixture side");
      load.force = load_element->Get<ignition::math::Vector3d>("force");
      load.offset = load_element->Get<ignition::math::Vector3d>("offset");
      load.torque = load_element->Get<ignition::math::Vector3d>("torque");
      load.start = load_element->Get<double>("start");
      load.duration = load_element->Get<double>("duration");
      loads_.push_back(load);
      load_element = load_element->GetNextElement("load");
    }
  }
  void PreUpdate(const sim::UpdateInfo &info,
                 sim::EntityComponentManager &ecm) override {
    if (info.paused)
      return;
    const double sim_seconds =
        std::chrono::duration<double>(info.simTime).count();
    for (auto &load : loads_) {
      // The world plugin can be configured before the robot model exists.
      if (load.link == sim::kNullEntity) {
        ecm.Each<sim::components::Model, sim::components::Name>(
            [&](const sim::Entity &entity, const sim::components::Model *,
                const sim::components::Name *name) {
              if (name->Data() == "riser")
                load.link = sim::Model(entity).LinkByName(
                    ecm, load.side + "_force_plate");
              return load.link == sim::kNullEntity;
            });
      }
      // Apply on [start, start + duration), once per unpaused physics step.
      if (load.link == sim::kNullEntity || sim_seconds < load.start ||
          sim_seconds >= load.start + load.duration)
        continue;
      sim::Link link(load.link);
      const auto pose = sim::worldPose(load.link, ecm);
      // Include the offset's moment r x F, then rotate both vectors to world.
      link.AddWorldWrench(
          ecm, pose.Rot().RotateVector(load.force),
          pose.Rot().RotateVector(load.torque + load.offset.Cross(load.force)));
    }
  }
};
} // namespace riser_gazebo
IGNITION_ADD_PLUGIN(riser_gazebo::LoadingFixture, ignition::gazebo::System,
                    riser_gazebo::LoadingFixture::ISystemConfigure,
                    riser_gazebo::LoadingFixture::ISystemPreUpdate)
