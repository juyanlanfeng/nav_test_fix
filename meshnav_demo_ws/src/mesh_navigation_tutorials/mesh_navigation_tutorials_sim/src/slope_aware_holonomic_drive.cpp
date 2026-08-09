// Copyright 2026 Nature Robots GmbH
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Nature Robots GmbH nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <ignition/common/Console.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/transport/Node.hh>

namespace mesh_navigation_tutorials_sim
{

class SlopeAwareHolonomicDrivePrivate
{
public:
  void OnCmdVel(const gz::msgs::Twist & msg)
  {
    std::lock_guard<std::mutex> lock(command_mutex);
    command = msg;
    new_command = true;
  }

  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Link canonical_link{gz::sim::kNullEntity};
  gz::transport::Node node;
  std::mutex command_mutex;
  gz::msgs::Twist command;
  bool new_command{false};
  bool command_received{false};
  std::chrono::steady_clock::duration last_command_time{0};
  std::chrono::steady_clock::duration command_timeout{
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(0.5))};
};

class SlopeAwareHolonomicDrive
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  SlopeAwareHolonomicDrive()
  : data_(std::make_unique<SlopeAwareHolonomicDrivePrivate>())
  {
  }

  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &) override
  {
    data_->model = gz::sim::Model(entity);
    if (!data_->model.Valid(ecm)) {
      ignerr << "SlopeAwareHolonomicDrive must be attached to a model.\n";
      return;
    }

    const double timeout = sdf->Get<double>("command_timeout", 0.5).first;
    data_->command_timeout =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout));

    std::string topic = "/model/" + data_->model.Name(ecm) + "/cmd_vel";
    if (sdf->HasElement("topic")) {
      topic = sdf->Get<std::string>("topic");
    }

    data_->node.Subscribe(
      topic, &SlopeAwareHolonomicDrivePrivate::OnCmdVel, data_.get());
    ignmsg << "SlopeAwareHolonomicDrive subscribing on [" << topic << "]\n";

    FindCanonicalLink(ecm);
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || !data_->model.Valid(ecm)) {
      return;
    }

    if (!data_->canonical_link.Valid(ecm) && !FindCanonicalLink(ecm)) {
      return;
    }

    gz::msgs::Twist command;
    bool command_received;
    {
      std::lock_guard<std::mutex> lock(data_->command_mutex);
      if (data_->new_command) {
        data_->last_command_time = info.simTime;
        data_->command_received = true;
        data_->new_command = false;
      }
      command = data_->command;
      command_received = data_->command_received;
    }

    if (!command_received ||
      (data_->command_timeout > std::chrono::steady_clock::duration::zero() &&
      info.simTime - data_->last_command_time > data_->command_timeout))
    {
      command.mutable_linear()->set_x(0.0);
      command.mutable_linear()->set_y(0.0);
      command.mutable_angular()->set_z(0.0);
    }

    const auto pose = data_->canonical_link.WorldPose(ecm);
    const auto world_linear = data_->canonical_link.WorldLinearVelocity(ecm);
    const auto world_angular = data_->canonical_link.WorldAngularVelocity(ecm);
    if (!pose || !world_linear || !world_angular) {
      return;
    }

    // Keep velocity components that are generated by terrain contact and
    // gravity. Only the three planar base degrees of freedom are commanded.
    auto body_linear = pose->Rot().RotateVectorReverse(*world_linear);
    auto body_angular = pose->Rot().RotateVectorReverse(*world_angular);
    body_linear.X(command.linear().x());
    body_linear.Y(command.linear().y());
    body_angular.Z(command.angular().z());

    SetCommand<gz::sim::components::LinearVelocityCmd>(
      ecm, data_->model.Entity(), body_linear);
    SetCommand<gz::sim::components::AngularVelocityCmd>(
      ecm, data_->model.Entity(), body_angular);
  }

private:
  bool FindCanonicalLink(gz::sim::EntityComponentManager & ecm)
  {
    const auto link_entity = data_->model.CanonicalLink(ecm);
    if (link_entity == gz::sim::kNullEntity) {
      return false;
    }
    data_->canonical_link = gz::sim::Link(link_entity);
    data_->canonical_link.EnableVelocityChecks(ecm);
    return true;
  }

  template<typename ComponentT>
  void SetCommand(
    gz::sim::EntityComponentManager & ecm,
    const gz::sim::Entity entity,
    const gz::math::Vector3d & velocity)
  {
    auto component = ecm.Component<ComponentT>(entity);
    if (component) {
      component->SetData(velocity, [](const auto &, const auto &) {return false;});
    } else {
      ecm.CreateComponent(entity, ComponentT(velocity));
    }
  }

  std::unique_ptr<SlopeAwareHolonomicDrivePrivate> data_;
};

}  // namespace mesh_navigation_tutorials_sim

IGNITION_ADD_PLUGIN(
  mesh_navigation_tutorials_sim::SlopeAwareHolonomicDrive,
  gz::sim::System,
  mesh_navigation_tutorials_sim::SlopeAwareHolonomicDrive::ISystemConfigure,
  mesh_navigation_tutorials_sim::SlopeAwareHolonomicDrive::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  mesh_navigation_tutorials_sim::SlopeAwareHolonomicDrive,
  "mesh_navigation_tutorials_sim::SlopeAwareHolonomicDrive")
