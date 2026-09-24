// SPDX-License-Identifier: Apache-2.0
// Simulation-only bounded velocity servo. No pose/velocity commands are used:
// the body remains subject to gravity, contacts, and engine integration.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/twist.pb.h>
#include <ignition/msgs/odometry.pb.h>

namespace pb_gazebo_sim_support
{
class VelocityDrive : public ignition::gazebo::System,
                      public ignition::gazebo::ISystemConfigure,
                      public ignition::gazebo::ISystemPreUpdate,
                      public ignition::gazebo::ISystemPostUpdate
{
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher odom_;
  ignition::gazebo::Entity model_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity link_{ignition::gazebo::kNullEntity};
  std::mutex mutex_;
  std::array<double, 3> target_{}, integral_{};
  // Match upstream force/torque limits; integral only removes steady-state error.
  std::array<double, 3> kp_{100., 500., 200.}, ki_{50., 100., 20.};
  std::array<double, 3> limit_{100., 200., 100.};
  std::chrono::steady_clock::time_point received_{};
  bool have_command_{false};

public:
  void Configure(const ignition::gazebo::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &,
                 ignition::gazebo::EntityComponentManager &,
                 ignition::gazebo::EventManager &) override
  {
    model_ = entity;
    node_.Subscribe("/robot/cmd_vel", &VelocityDrive::OnCommand, this);
    odom_ = node_.Advertise<ignition::msgs::Odometry>("/robot/odometry");
  }

  void OnCommand(const ignition::msgs::Twist &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::array<double, 3> values{msg.linear().x(), msg.linear().y(), msg.angular().z()};
    for (double v : values) if (!std::isfinite(v)) return;
    target_ = values;
    received_ = std::chrono::steady_clock::now();
    have_command_ = true;
  }

  void PreUpdate(const ignition::gazebo::UpdateInfo &info,
                 ignition::gazebo::EntityComponentManager &ecm) override
  {
    using namespace ignition::gazebo;
    using namespace ignition::gazebo::components;
    if (link_ == kNullEntity) {
      link_ = Model(model_).LinkByName(ecm, "chassis");
      if (link_ == kNullEntity) return;
      Link(link_).EnableVelocityChecks(ecm);
      if (!ecm.Component<WorldPose>(link_)) ecm.CreateComponent(link_, WorldPose());
      return;
    }
    if (info.paused) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0. || dt > .1) {integral_.fill(0.); return;}
    const auto pose = Link(link_).WorldPose(ecm);
    const auto vel = Link(link_).WorldLinearVelocity(ecm);
    const auto omega = Link(link_).WorldAngularVelocity(ecm);
    if (!pose || !vel || !omega) return;
    std::array<double, 3> target;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool fresh = have_command_ &&
          std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() < .5;
      target = fresh ? target_ : std::array<double, 3>{};
      if (!fresh) integral_.fill(0.);
    }
    const auto body_v = pose->Rot().RotateVectorReverse(*vel);
    const auto body_w = pose->Rot().RotateVectorReverse(*omega);
    const std::array<double, 3> measured{body_v.X(), body_v.Y(), body_w.Z()};
    std::array<double, 3> effort{};
    for (size_t i = 0; i < 3; ++i) {
      const double error = target[i] - measured[i];
      const double proposed = std::clamp(integral_[i] + ki_[i] * error * dt, -limit_[i], limit_[i]);
      const double raw = kp_[i] * error + proposed;
      // Conditional integration: do not wind up against the effort cap.
      if (std::abs(raw) <= limit_[i] || raw * error < 0.) integral_[i] = proposed;
      effort[i] = std::clamp(kp_[i] * error + integral_[i], -limit_[i], limit_[i]);
    }
    Link(link_).AddWorldWrench(ecm, pose->Rot().RotateVector({effort[0], effort[1], 0.}),
                              pose->Rot().RotateVector({0., 0., effort[2]}));
  }

  void PostUpdate(const ignition::gazebo::UpdateInfo &info,
                  const ignition::gazebo::EntityComponentManager &ecm) override
  {
    if (info.paused || link_ == ignition::gazebo::kNullEntity) return;
    const auto link = ignition::gazebo::Link(link_);
    const auto p = link.WorldPose(ecm);
    const auto v = link.WorldLinearVelocity(ecm);
    const auto w = link.WorldAngularVelocity(ecm);
    if (!p || !v || !w) return;
    ignition::msgs::Odometry msg;
    *msg.mutable_header()->mutable_stamp() = ignition::gazebo::convert<ignition::msgs::Time>(info.simTime);
    auto f = msg.mutable_header()->add_data(); f->set_key("frame_id"); f->add_value("rmuc2026_field");
    auto c = msg.mutable_header()->add_data(); c->set_key("child_frame_id"); c->add_value("chassis");
    ignition::msgs::Set(msg.mutable_pose(), *p);
    ignition::msgs::Set(msg.mutable_twist()->mutable_linear(), p->Rot().RotateVectorReverse(*v));
    ignition::msgs::Set(msg.mutable_twist()->mutable_angular(), p->Rot().RotateVectorReverse(*w));
    odom_.Publish(msg);
  }
};
}
IGNITION_ADD_PLUGIN(pb_gazebo_sim_support::VelocityDrive, ignition::gazebo::System,
    pb_gazebo_sim_support::VelocityDrive::ISystemConfigure,
    pb_gazebo_sim_support::VelocityDrive::ISystemPreUpdate,
    pb_gazebo_sim_support::VelocityDrive::ISystemPostUpdate)
