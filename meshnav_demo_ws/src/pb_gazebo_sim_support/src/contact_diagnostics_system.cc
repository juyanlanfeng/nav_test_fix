// Copyright 2026. Licensed under Apache-2.0.
//
// ContactDiagnostics: read-only contact and actuator diagnostics for the PB vehicle.
//
// Why this exists (doc/PB_SLOPE_NEXT_STEPS_20260921.md P1): the plan requires
// contact/force instrumentation *before* any repair, and forbids changing physics
// to make the symptom go away.  The earlier evidence could only discriminate
// "held chassis" from "wheel slip" indirectly, from ROS wheel velocities; it could
// not show the wrench the drive actually asked for, nor the joint forces and
// contact points that decide between a geometric block and an actuator shortfall.
//
// Requests engine state components and republishes their measured data. Creating
// ContactSensorData on a collision opts into Physics contact reporting; it is not
// a force/pose command. Mass, friction, geometry and gravity remain unchanged.
//
// Default off, explicit switch.  `<enable>` defaults to FALSE: with the element
// absent, or set to anything but "true", Configure() logs one line and the system
// never creates a component, never advertises a topic and never publishes.  Turning
// it on is a deliberate edit of one element in the model.
//
// SDF parameters:
//   <enable>            master switch, FALSE by default         (false)
//   <enable_actuator>   requested wrench + joint state          (true)
//   <enable_contact>    republish contact sensor data           (true)
//   <model_name>        model to scope the search to, only needed when the plugin
//                       is attached to the world                    ("")
//   <link_name>         chassis link whose wrench is read       (chassis)
//   <joints>            whitespace/comma separated joint names.  Defaults to the
//                       four wheels
//   <topic_prefix>      topic prefix                (/pb_sim/diagnostics)
//   <publish_period_s>  > 0 throttles publication               (0.05)
//
// Topics (Gazebo transport, so they are visible to `ign topic -e`):
//   <prefix>/requested_wrench  ignition.msgs.Wrench
//       The body wrench MecanumDrive2 asked the physics engine to apply (force and
//       torque in the world frame).  This is the requested actuator effort, not the
//       delivered one.  ExternalWorldWrenchCmd is a per-step command that the engine
//       clears, so it is sampled in BOTH PreUpdate and PostUpdate and the largest
//       magnitude seen since the last publication is reported.  Reading it only in
//       PostUpdate silently yields an all-zero stream, which is indistinguishable
//       from "the drive never asked for anything" — the exact confusion this
//       diagnostic exists to remove.
//   <prefix>/joint_state       ignition.msgs.Float_V
//       Layout: [velocity * N, position * N, applied_force * N], joints in the
//       order given by <joint>.  Applied force is the engine's JointForce.
//   <prefix>/contact           ignition.msgs.Contacts
//       Engine contact data for collisions belonging to this model. Some engines
//       provide positions only: missing normal/depth/wrench fields are NOT zeros
//       and cannot be used to conclude that there is no contact force.
//
// A throttled text line is also logged, because the evidence pipeline captures the
// server log and a human-readable trace is easier to audit than a binary topic.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/ExternalWorldWrenchCmd.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointForce.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/contact.pb.h>
#include <ignition/msgs/float_v.pb.h>
#include <ignition/msgs/wrench.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Element.hh>

#include "pb_gazebo_sim_support/diagnostics_logic.h"

namespace pb_gazebo_sim_support
{
class ContactDiagnosticsPrivate;

/// \brief Read-only contact and actuator diagnostics.  See the file header.
class ContactDiagnostics : public ignition::gazebo::System,
                           public ignition::gazebo::ISystemConfigure,
                           public ignition::gazebo::ISystemPreUpdate,
                           public ignition::gazebo::ISystemPostUpdate
{
  public: ContactDiagnostics();
  public: ~ContactDiagnostics() override;

  public: void Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr) override;

  public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                         ignition::gazebo::EntityComponentManager &_ecm) override;

  public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                          const ignition::gazebo::EntityComponentManager &_ecm) override;

  private: std::unique_ptr<ContactDiagnosticsPrivate> dataPtr;
};

class ContactDiagnosticsPrivate
{
public:
  ignition::transport::Node node;
  ignition::transport::Node::Publisher wrenchPub;
  ignition::transport::Node::Publisher jointPub;
  ignition::transport::Node::Publisher contactPub;

  bool enabled{false};
  bool enableActuator{true};
  bool enableContact{true};

  ignition::gazebo::Entity modelEntity{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity linkEntity{ignition::gazebo::kNullEntity};
  std::string modelName;
  std::string linkName{"chassis"};
  std::string topicPrefix{"/pb_sim/diagnostics"};
  std::vector<std::string> jointNames{
      "front_left_wheel_joint", "front_right_wheel_joint",
      "rear_left_wheel_joint", "rear_right_wheel_joint"};
  std::vector<ignition::gazebo::Entity> jointEntities;
  std::vector<ignition::gazebo::Entity> contactSensorEntities;

  double publishPeriodS{0.05};
  double lastPublishS{-1.0};
  double lastLogS{-1.0};

  /// \brief Largest requested wrench magnitude seen since the last publication.
  /// The command component is cleared every step, so it must be sampled wherever it
  /// is visible rather than assumed present at publication time.
  ignition::msgs::Wrench peakWrench;
  double peakWrenchForce{0.0};

  bool reportedResolution{false};
  bool warnedNoContactSensor{false};
  bool warnedNoContactData{false};
  bool warnedNoRequestedWrench{false};

  /// \brief Fold the current ExternalWorldWrenchCmd, if any, into the peak.
  template <typename ECM>
  void SampleRequestedWrench(ECM &_ecm)
  {
    const auto *wrench =
        _ecm.template Component<ignition::gazebo::components::ExternalWorldWrenchCmd>(
            this->linkEntity);
    if (wrench == nullptr)
    {
      if (!this->warnedNoRequestedWrench)
      {
        ignwarn << "ContactDiagnostics: no ExternalWorldWrenchCmd on link ["
                << this->linkName
                << "]; the drive plugin may not be applying a body wrench.  Requested "
                   "effort will report zero." << std::endl;
        this->warnedNoRequestedWrench = true;
      }
      return;
    }
    const auto &force = wrench->Data().force();
    const double magnitude = std::sqrt(force.x() * force.x() +
                                       force.y() * force.y() +
                                       force.z() * force.z());
    if (magnitude > this->peakWrenchForce)
    {
      this->peakWrenchForce = magnitude;
      *this->peakWrench.mutable_force() = force;
      *this->peakWrench.mutable_torque() = wrench->Data().torque();
    }
  }
};

ContactDiagnostics::ContactDiagnostics()
    : dataPtr(std::make_unique<ContactDiagnosticsPrivate>())
{
}

//////////////////////////////////////////////////
ContactDiagnostics::~ContactDiagnostics() = default;

//////////////////////////////////////////////////
void ContactDiagnostics::Configure(const ignition::gazebo::Entity &_entity,
                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                   ignition::gazebo::EntityComponentManager &_ecm,
                                   ignition::gazebo::EventManager & /*_eventMgr*/)
{
  // Default off.  Nothing below runs unless the model explicitly switches it on.
  this->dataPtr->enabled =
      DiagnosticsEnabledFromElement(_sdf, "enable", false);
  if (!this->dataPtr->enabled)
  {
    ignmsg << "ContactDiagnostics: disabled (set <enable>true</enable> to collect "
              "read-only contact/actuator diagnostics)" << std::endl;
    return;
  }
  this->dataPtr->enableActuator =
      DiagnosticsEnabledFromElement(_sdf, "enable_actuator", true);
  this->dataPtr->enableContact =
      DiagnosticsEnabledFromElement(_sdf, "enable_contact", true);

  if (_sdf->HasElement("model_name"))
  {
    this->dataPtr->modelName = _sdf->Get<std::string>("model_name");
  }
  if (_sdf->HasElement("link_name"))
  {
    this->dataPtr->linkName = _sdf->Get<std::string>("link_name");
  }
  if (_sdf->HasElement("topic_prefix"))
  {
    this->dataPtr->topicPrefix = _sdf->Get<std::string>("topic_prefix");
  }
  if (_sdf->HasElement("publish_period_s"))
  {
    this->dataPtr->publishPeriodS = _sdf->Get<double>("publish_period_s");
  }
  if (_sdf->HasElement("joints"))
  {
    const auto names = ParseJointList(_sdf->Get<std::string>("joints"));
    if (!names.empty())
    {
      this->dataPtr->jointNames = names;
    }
  }

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

  if (this->dataPtr->enableActuator)
  {
    this->dataPtr->wrenchPub = this->dataPtr->node.Advertise<ignition::msgs::Wrench>(
        this->dataPtr->topicPrefix + "/requested_wrench");
    this->dataPtr->jointPub = this->dataPtr->node.Advertise<ignition::msgs::Float_V>(
        this->dataPtr->topicPrefix + "/joint_state");
  }
  if (this->dataPtr->enableContact)
  {
    this->dataPtr->contactPub = this->dataPtr->node.Advertise<ignition::msgs::Contacts>(
        this->dataPtr->topicPrefix + "/contact");
  }

  ignmsg << "ContactDiagnostics: ENABLED (read-only). actuator="
         << (this->dataPtr->enableActuator ? "on" : "off") << " contact="
         << (this->dataPtr->enableContact ? "on" : "off") << " prefix ["
         << this->dataPtr->topicPrefix << "] link [" << this->dataPtr->linkName
         << "] joints [" << this->dataPtr->jointNames.size() << "]" << std::endl;
}

//////////////////////////////////////////////////
void ContactDiagnostics::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                                   ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!this->dataPtr->enabled || _info.paused)
  {
    return;
  }

  // Late resolution, scoped to the configured model exactly like ChassisTruth.
  if (this->dataPtr->linkEntity == ignition::gazebo::kNullEntity &&
      this->dataPtr->modelEntity != ignition::gazebo::kNullEntity)
  {
    ignition::gazebo::Model model(this->dataPtr->modelEntity);
    if (model.Valid(_ecm))
    {
      this->dataPtr->linkEntity = model.LinkByName(_ecm, this->dataPtr->linkName);
    }
  }
  if (this->dataPtr->linkEntity == ignition::gazebo::kNullEntity)
  {
    return;
  }
  if (this->dataPtr->jointEntities.size() != this->dataPtr->jointNames.size())
  {
    this->dataPtr->jointEntities.clear();
    for (const auto &name : this->dataPtr->jointNames)
    {
      this->dataPtr->jointEntities.push_back(_ecm.EntityByComponents(
          ignition::gazebo::components::Joint(),
          ignition::gazebo::components::Name(name)));
    }
  }

  if (!this->dataPtr->reportedResolution)
  {
    ignmsg << "ContactDiagnostics: resolved link [" << this->dataPtr->linkName
           << "] entity [" << this->dataPtr->linkEntity << "]" << std::endl;
    this->dataPtr->reportedResolution = true;
  }

  // Sample the requested wrench here as well as in PostUpdate: it is a per-step
  // command that the engine clears, so neither read point alone is guaranteed to
  // catch it.
  this->dataPtr->SampleRequestedWrench(_ecm);

  // Ask the engine for the quantities read in PostUpdate.  Creating a *state*
  // component only makes the engine compute and expose it; none of these are
  // commands, so this cannot change the simulation.
  // Only state components are requested: joint state is what the diagnostic
  // publishes, and none of these are command components, so this cannot alter the
  // simulation.  The requested wrench is deliberately NOT created here — it is read
  // only if the drive plugin already writes it, so an absent wrench is reported as
  // absent instead of being manufactured as a zero sample.
  using namespace ignition::gazebo::components;
  for (auto jointEntity : this->dataPtr->jointEntities)
  {
    if (jointEntity == ignition::gazebo::kNullEntity)
    {
      continue;
    }
    if (_ecm.Component<JointVelocity>(jointEntity) == nullptr)
    {
      _ecm.CreateComponent(jointEntity, JointVelocity());
    }
    if (_ecm.Component<JointPosition>(jointEntity) == nullptr)
    {
      _ecm.CreateComponent(jointEntity, JointPosition());
    }
    if (_ecm.Component<JointForce>(jointEntity) == nullptr)
    {
      _ecm.CreateComponent(jointEntity, JointForce());
    }
  }

  if (this->dataPtr->enableContact && this->dataPtr->contactSensorEntities.empty())
  {
    // Collision -> link -> model: never collect another robot's contacts.
    _ecm.Each<ignition::gazebo::components::Collision>(
        [&](const ignition::gazebo::Entity &_sensorEntity,
            ignition::gazebo::components::Collision * /*_collision*/)
        {
          const auto *parent = _ecm.Component<ignition::gazebo::components::ParentEntity>(
              _sensorEntity);
          const auto *modelParent = parent == nullptr ? nullptr :
              _ecm.Component<ignition::gazebo::components::ParentEntity>(parent->Data());
          if (modelParent != nullptr && modelParent->Data() == this->dataPtr->modelEntity)
          {
            this->dataPtr->contactSensorEntities.push_back(_sensorEntity);
          }
          return true;
        });
    if (this->dataPtr->contactSensorEntities.empty() &&
        !this->dataPtr->warnedNoContactSensor)
    {
      ignwarn << "ContactDiagnostics: contact diagnostics are on but the model has "
                 "no collision entities; waiting for model creation." << std::endl;
      this->dataPtr->warnedNoContactSensor = true;
    }
  }
  // Physics fills this state component on COLLISIONS (not sensor entities).
  // Requesting it is the same opt-in used by Fortress's Contact system and does
  // not alter any force or collision geometry. It also supports runtime spawns.
  for (const auto entity : this->dataPtr->contactSensorEntities)
  {
    if (!_ecm.Component<ignition::gazebo::components::ContactSensorData>(entity))
      _ecm.CreateComponent(entity, ignition::gazebo::components::ContactSensorData());
  }
}

//////////////////////////////////////////////////
void ContactDiagnostics::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                                    const ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!this->dataPtr->enabled || _info.paused ||
      this->dataPtr->linkEntity == ignition::gazebo::kNullEntity)
  {
    return;
  }

  const double simSeconds = std::chrono::duration<double>(_info.simTime).count();
  if (this->dataPtr->publishPeriodS > 0.0 && this->dataPtr->lastPublishS >= 0.0 &&
      simSeconds - this->dataPtr->lastPublishS < this->dataPtr->publishPeriodS)
  {
    return;
  }
  this->dataPtr->lastPublishS = simSeconds;

  double wrenchForceMagnitude = 0.0;

  // ---- requested actuator effort: the peak wrench seen since the last publish.
  if (this->dataPtr->enableActuator)
  {
    this->dataPtr->SampleRequestedWrench(_ecm);
    this->dataPtr->wrenchPub.Publish(this->dataPtr->peakWrench);
    wrenchForceMagnitude = this->dataPtr->peakWrenchForce;
    this->dataPtr->peakWrench.Clear();
    this->dataPtr->peakWrenchForce = 0.0;
  }

  // ---- joint state: velocity, position and applied force per configured joint.
  std::vector<double> velocities;
  std::vector<double> positions;
  std::vector<double> forces;
  for (auto jointEntity : this->dataPtr->jointEntities)
  {
    if (jointEntity == ignition::gazebo::kNullEntity)
    {
      velocities.push_back(0.0);
      positions.push_back(0.0);
      forces.push_back(0.0);
      continue;
    }
    const auto *velocity =
        _ecm.Component<ignition::gazebo::components::JointVelocity>(jointEntity);
    const auto *position =
        _ecm.Component<ignition::gazebo::components::JointPosition>(jointEntity);
    const auto *force =
        _ecm.Component<ignition::gazebo::components::JointForce>(jointEntity);
    velocities.push_back(velocity == nullptr || velocity->Data().empty()
                             ? 0.0 : velocity->Data()[0]);
    positions.push_back(position == nullptr || position->Data().empty()
                            ? 0.0 : position->Data()[0]);
    forces.push_back(force == nullptr || force->Data().empty()
                         ? 0.0 : force->Data()[0]);
  }
  if (this->dataPtr->enableActuator)
  {
    ignition::msgs::Float_V msg;
    for (double value : JointStateLayout(velocities, positions, forces))
    {
      msg.add_data(static_cast<float>(value));
    }
    this->dataPtr->jointPub.Publish(msg);
  }

  // ---- contacts supplied by Physics on the requested collision components.
  bool haveContact = false;
  if (this->dataPtr->enableContact)
  {
    for (auto sensorEntity : this->dataPtr->contactSensorEntities)
    {
      const auto *contacts =
          _ecm.Component<ignition::gazebo::components::ContactSensorData>(sensorEntity);
      if (contacts == nullptr)
      {
        continue;
      }
      auto sample = contacts->Data();
      *sample.mutable_header()->mutable_stamp() =
          ignition::gazebo::convert<ignition::msgs::Time>(_info.simTime);
      this->dataPtr->contactPub.Publish(sample);
      if (contacts->Data().contact_size() > 0)
      {
        haveContact = true;
      }
    }
    if (!haveContact && !this->dataPtr->warnedNoContactData)
    {
      ignwarn << "ContactDiagnostics: no nonempty contact samples yet; "
                 "the model may be airborne or the engine has not supplied data." << std::endl;
      this->dataPtr->warnedNoContactData = true;
    }
  }

  // ---- one throttled human-readable line: the trace the report quotes.
  if (this->dataPtr->lastLogS < 0.0 || simSeconds - this->dataPtr->lastLogS >= 1.0)
  {
    this->dataPtr->lastLogS = simSeconds;
    ignmsg << "ContactDiagnostics t=" << simSeconds
           << " requested_force=" << wrenchForceMagnitude << " joints=";
    for (std::size_t i = 0; i < velocities.size(); ++i)
    {
      ignmsg << this->dataPtr->jointNames[i] << "(v=" << velocities[i]
             << ",f=" << forces[i] << ") ";
    }
    ignmsg << "contacts=" << (haveContact ? "yes" : "none") << std::endl;
  }
}
}  // namespace pb_gazebo_sim_support

IGNITION_ADD_PLUGIN(
    pb_gazebo_sim_support::ContactDiagnostics,
    ignition::gazebo::System,
    pb_gazebo_sim_support::ContactDiagnostics::ISystemConfigure,
    pb_gazebo_sim_support::ContactDiagnostics::ISystemPreUpdate,
    pb_gazebo_sim_support::ContactDiagnostics::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    pb_gazebo_sim_support::ContactDiagnostics,
    "ignition::gazebo::systems::ContactDiagnostics")
