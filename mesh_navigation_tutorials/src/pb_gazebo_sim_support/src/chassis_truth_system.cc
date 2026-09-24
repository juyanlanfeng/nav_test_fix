// Copyright 2026. Licensed under Apache-2.0.
//
// ChassisTruth: publish same-step, simulation-time-stamped six-degree-of-freedom
// chassis ground truth.
//
// Why this exists (doc/PB_SLOPE_REPAIR_AND_DEPLOYMENT_PLAN.md section 4): the
// navigation TF/odometry chain was composed from two asynchronous ROS streams
// (world pose from SceneBroadcaster and model->chassis from PosePublisher).  Their
// stamps disagree, so the composed result could either be stale (the world pose
// stopped updating while the chassis stream kept going) or carry another
// message's time.  The drive plugin's own odometry is a single step and is
// sim-time stamped, but it publishes yaw-only orientation and no z position, so
// it cannot represent a chassis on a ramp.
//
// This system reads the chassis WorldPose together with the engine velocities in
// one PostUpdate, expresses pose in the world frame and twist in the chassis frame
// (the nav_msgs/Odometry / ignition.msgs.Odometry contract), and stamps the
// message with that step's simulation time.  One message is therefore a complete,
// same-instant measurement and never needs to be fused with another stream.
//
// SDF parameters (all optional except the link):
//   <link_name>          chassis link name                          (chassis)
//   <model_name>         model to scope the search to, only needed when the
//                        plugin is attached to the world               ("")
//   <topic>              Gazebo transport topic   (/pb_sim/chassis_truth)
//   <world_frame>        frame_id for the pose        (rmuc2026_field)
//   <child_frame>        child_frame_id for the twist         (chassis)
//   <publish_period_s>   0 publishes every step, > 0 throttles        (0.0)

#include <chrono>
#include <memory>
#include <string>

#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/odometry.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Element.hh>

namespace pb_gazebo_sim_support
{
class ChassisTruthPrivate;

/// \brief Publishes same-step, simulation-time-stamped 6-DOF chassis truth.
class ChassisTruth : public ignition::gazebo::System,
                     public ignition::gazebo::ISystemConfigure,
                     public ignition::gazebo::ISystemPreUpdate,
                     public ignition::gazebo::ISystemPostUpdate
{
  public: ChassisTruth();
  public: ~ChassisTruth() override;

  public: void Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr) override;

  public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm) override;

  public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                          const ignition::gazebo::EntityComponentManager &_ecm) override;

  private: std::unique_ptr<ChassisTruthPrivate> dataPtr;
};

class ChassisTruthPrivate
{
public:
  /// \brief Gazebo transport node used for publishing.
  ignition::transport::Node node;

  /// \brief Odometry publisher.
  ignition::transport::Node::Publisher publisher;

  /// \brief Model scoping the link search; kNullEntity until resolved.
  ignition::gazebo::Entity modelEntity{ignition::gazebo::kNullEntity};

  /// \brief Chassis link entity; kNullEntity until resolved.
  ignition::gazebo::Entity linkEntity{ignition::gazebo::kNullEntity};

  /// \brief Configured names.
  std::string modelName;
  std::string linkName{"chassis"};

  /// \brief Message frame ids.
  std::string worldFrame{"rmuc2026_field"};
  std::string childFrame{"chassis"};

  /// \brief 0 = publish every step.
  double publishPeriodS{0.0};

  /// \brief Simulation time of the last published sample.
  double lastPublishS{-1.0};

  /// \brief Whether the "no engine velocity component" warning was emitted.
  bool warnedNoEngineVelocity{false};

  /// \brief Whether the "resolved link" message was emitted.
  bool reportedResolution{false};
};

ChassisTruth::ChassisTruth() : dataPtr(std::make_unique<ChassisTruthPrivate>())
{
}

//////////////////////////////////////////////////
ChassisTruth::~ChassisTruth() = default;

//////////////////////////////////////////////////
void ChassisTruth::Configure(const ignition::gazebo::Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             ignition::gazebo::EntityComponentManager &_ecm,
                             ignition::gazebo::EventManager & /*_eventMgr*/)
{
  if (_sdf->HasElement("link_name"))
  {
    this->dataPtr->linkName = _sdf->Get<std::string>("link_name");
  }
  if (_sdf->HasElement("model_name"))
  {
    this->dataPtr->modelName = _sdf->Get<std::string>("model_name");
  }
  if (_sdf->HasElement("world_frame"))
  {
    this->dataPtr->worldFrame = _sdf->Get<std::string>("world_frame");
  }
  if (_sdf->HasElement("child_frame"))
  {
    this->dataPtr->childFrame = _sdf->Get<std::string>("child_frame");
  }
  if (_sdf->HasElement("publish_period_s"))
  {
    this->dataPtr->publishPeriodS = _sdf->Get<double>("publish_period_s");
  }
  std::string topic{"/pb_sim/chassis_truth"};
  if (_sdf->HasElement("topic"))
  {
    topic = _sdf->Get<std::string>("topic");
  }

  // Prefer the entity the plugin is attached to.  Only when that is not a model
  // (for example the plugin is attached to the world) fall back to a *named*
  // model lookup; never search a bare link name across the world, which would be
  // ambiguous with several robots.
  if (_ecm.Component<ignition::gazebo::components::Model>(_entity) != nullptr)
  {
    this->dataPtr->modelEntity = _entity;
  }
  else if (!this->dataPtr->modelName.empty())
  {
    this->dataPtr->modelEntity = _ecm.EntityByComponents(
        ignition::gazebo::components::Model(),
        ignition::gazebo::components::Name(this->dataPtr->modelName));
  }

  if (this->dataPtr->modelEntity != ignition::gazebo::kNullEntity)
  {
    ignition::gazebo::Model model(this->dataPtr->modelEntity);
    if (model.Valid(_ecm))
    {
      this->dataPtr->linkEntity = model.LinkByName(_ecm, this->dataPtr->linkName);
    }
  }

  this->dataPtr->publisher =
      this->dataPtr->node.Advertise<ignition::msgs::Odometry>(topic);

  ignmsg << "ChassisTruth: publishing same-step 6-DOF chassis truth on [" << topic
         << "] link [" << this->dataPtr->linkName << "] world_frame ["
         << this->dataPtr->worldFrame << "] child_frame ["
         << this->dataPtr->childFrame << "]" << std::endl;
  if (this->dataPtr->linkEntity == ignition::gazebo::kNullEntity)
  {
    ignwarn << "ChassisTruth: chassis link [" << this->dataPtr->linkName
            << "] is not resolved yet; it will be retried every step within the"
            << " configured model scope." << std::endl;
  }
}

//////////////////////////////////////////////////
void ChassisTruth::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                             ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
  {
    return;
  }

  // Resolve late (a model may be created after Configure, and links can be added
  // dynamically).  The lookup stays scoped to the configured model entity.
  if (this->dataPtr->linkEntity == ignition::gazebo::kNullEntity)
  {
    if (this->dataPtr->modelEntity == ignition::gazebo::kNullEntity &&
        !this->dataPtr->modelName.empty())
    {
      this->dataPtr->modelEntity = _ecm.EntityByComponents(
          ignition::gazebo::components::Model(),
          ignition::gazebo::components::Name(this->dataPtr->modelName));
    }
    if (this->dataPtr->modelEntity == ignition::gazebo::kNullEntity)
    {
      return;
    }
    ignition::gazebo::Model model(this->dataPtr->modelEntity);
    if (!model.Valid(_ecm))
    {
      return;
    }
    this->dataPtr->linkEntity = model.LinkByName(_ecm, this->dataPtr->linkName);
    if (this->dataPtr->linkEntity == ignition::gazebo::kNullEntity)
    {
      return;
    }
  }

  if (!this->dataPtr->reportedResolution)
  {
    ignmsg << "ChassisTruth: resolved chassis link [" << this->dataPtr->linkName
           << "] in model entity [" << this->dataPtr->modelEntity << "]"
           << std::endl;
    this->dataPtr->reportedResolution = true;
  }

  // Request exactly the quantities read in PostUpdate.  Pose and world-frame
  // velocities are what make the message complete; the link-frame variants are a
  // fallback for builds that do not populate the world-frame components.
  using namespace ignition::gazebo::components;
  if (_ecm.Component<WorldPose>(this->dataPtr->linkEntity) == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->linkEntity, WorldPose());
  }
  if (_ecm.Component<WorldLinearVelocity>(this->dataPtr->linkEntity) == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->linkEntity, WorldLinearVelocity());
  }
  if (_ecm.Component<WorldAngularVelocity>(this->dataPtr->linkEntity) == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->linkEntity, WorldAngularVelocity());
  }
  if (_ecm.Component<LinearVelocity>(this->dataPtr->linkEntity) == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->linkEntity, LinearVelocity());
  }
  if (_ecm.Component<AngularVelocity>(this->dataPtr->linkEntity) == nullptr)
  {
    _ecm.CreateComponent(this->dataPtr->linkEntity, AngularVelocity());
  }
}

//////////////////////////////////////////////////
void ChassisTruth::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                              const ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused || this->dataPtr->linkEntity == ignition::gazebo::kNullEntity)
  {
    return;
  }

  const double simSeconds =
      std::chrono::duration<double>(_info.simTime).count();
  if (this->dataPtr->publishPeriodS > 0.0 &&
      this->dataPtr->lastPublishS >= 0.0 &&
      simSeconds - this->dataPtr->lastPublishS < this->dataPtr->publishPeriodS)
  {
    return;
  }

  using namespace ignition::gazebo::components;
  const auto *poseComp = _ecm.Component<WorldPose>(this->dataPtr->linkEntity);
  if (poseComp == nullptr)
  {
    return;
  }
  const ignition::math::Pose3d &pose = poseComp->Data();
  if (!pose.IsFinite())
  {
    ignwarn << "ChassisTruth: non-finite chassis pose at sim " << simSeconds
            << "; sample dropped" << std::endl;
    return;
  }

  // Twist must be expressed in the child (chassis) frame.  Prefer the engine's
  // world-frame velocities and rotate them; fall back to the link-frame
  // components, which are already in the chassis frame.
  ignition::math::Vector3d linearBody;
  ignition::math::Vector3d angularBody;
  const auto *worldLinear = _ecm.Component<WorldLinearVelocity>(this->dataPtr->linkEntity);
  const auto *worldAngular = _ecm.Component<WorldAngularVelocity>(this->dataPtr->linkEntity);
  const auto *linkLinear = _ecm.Component<LinearVelocity>(this->dataPtr->linkEntity);
  const auto *linkAngular = _ecm.Component<AngularVelocity>(this->dataPtr->linkEntity);
  if (worldLinear != nullptr && worldAngular != nullptr &&
      worldLinear->Data().IsFinite() && worldAngular->Data().IsFinite())
  {
    linearBody = pose.Rot().RotateVectorReverse(worldLinear->Data());
    angularBody = pose.Rot().RotateVectorReverse(worldAngular->Data());
  }
  else if (linkLinear != nullptr && linkAngular != nullptr &&
           linkLinear->Data().IsFinite() && linkAngular->Data().IsFinite())
  {
    linearBody = linkLinear->Data();
    angularBody = linkAngular->Data();
  }
  else if (!this->dataPtr->warnedNoEngineVelocity)
  {
    ignwarn << "ChassisTruth: no engine velocity component available on link ["
            << this->dataPtr->linkName
            << "]; publishing zero twist (position and orientation are still"
            << " exact for this step)" << std::endl;
    this->dataPtr->warnedNoEngineVelocity = true;
  }

  ignition::msgs::Odometry msg;
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      ignition::gazebo::convert<ignition::msgs::Time>(_info.simTime));
  auto *frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->dataPtr->worldFrame);
  auto *child = msg.mutable_header()->add_data();
  child->set_key("child_frame_id");
  child->add_value(this->dataPtr->childFrame);

  msg.mutable_pose()->mutable_position()->set_x(pose.X());
  msg.mutable_pose()->mutable_position()->set_y(pose.Y());
  msg.mutable_pose()->mutable_position()->set_z(pose.Z());
  ignition::msgs::Set(msg.mutable_pose()->mutable_orientation(), pose.Rot());

  msg.mutable_twist()->mutable_linear()->set_x(linearBody.X());
  msg.mutable_twist()->mutable_linear()->set_y(linearBody.Y());
  msg.mutable_twist()->mutable_linear()->set_z(linearBody.Z());
  msg.mutable_twist()->mutable_angular()->set_x(angularBody.X());
  msg.mutable_twist()->mutable_angular()->set_y(angularBody.Y());
  msg.mutable_twist()->mutable_angular()->set_z(angularBody.Z());

  this->dataPtr->publisher.Publish(msg);
  this->dataPtr->lastPublishS = simSeconds;
}
}  // namespace pb_gazebo_sim_support

IGNITION_ADD_PLUGIN(
    pb_gazebo_sim_support::ChassisTruth,
    ignition::gazebo::System,
    pb_gazebo_sim_support::ChassisTruth::ISystemConfigure,
    pb_gazebo_sim_support::ChassisTruth::ISystemPreUpdate,
    pb_gazebo_sim_support::ChassisTruth::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    pb_gazebo_sim_support::ChassisTruth,
    "ignition::gazebo::systems::ChassisTruth")
