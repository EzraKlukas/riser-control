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
    auto el = sdf->FindElement("load");
    while (el) {
      Load l;
      l.side = el->Get<std::string>("side");
      if (l.side != "left" && l.side != "right")
        throw std::runtime_error("Invalid fixture side");
      l.force = el->Get<ignition::math::Vector3d>("force");
      l.offset = el->Get<ignition::math::Vector3d>("offset");
      l.torque = el->Get<ignition::math::Vector3d>("torque");
      l.start = el->Get<double>("start");
      l.duration = el->Get<double>("duration");
      loads_.push_back(l);
      el = el->GetNextElement("load");
    }
  }
  void PreUpdate(const sim::UpdateInfo &info,
                 sim::EntityComponentManager &ecm) override {
    if (info.paused)
      return;
    const double t = std::chrono::duration<double>(info.simTime).count();
    for (auto &l : loads_) {
      if (l.link == sim::kNullEntity) {
        ecm.Each<sim::components::Model, sim::components::Name>(
            [&](const sim::Entity &e, const sim::components::Model *,
                const sim::components::Name *n) {
              if (n->Data() == "riser")
                l.link = sim::Model(e).LinkByName(ecm, l.side + "_force_plate");
              return l.link == sim::kNullEntity;
            });
      }
      if (l.link == sim::kNullEntity || t < l.start ||
          t >= l.start + l.duration)
        continue;
      sim::Link link(l.link);
      const auto pose = sim::worldPose(l.link, ecm);
      link.AddWorldWrench(
          ecm, pose.Rot().RotateVector(l.force),
          pose.Rot().RotateVector(l.torque + l.offset.Cross(l.force)));
    }
  }
};
} // namespace riser_gazebo
IGNITION_ADD_PLUGIN(riser_gazebo::LoadingFixture, ignition::gazebo::System,
                    riser_gazebo::LoadingFixture::ISystemConfigure,
                    riser_gazebo::LoadingFixture::ISystemPreUpdate)
