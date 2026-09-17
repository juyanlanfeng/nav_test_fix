// Physics-only regression client. No ROS or graphics required.
#include <ignition/transport/Node.hh>
#include <ignition/msgs/twist.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <atomic>
#include <chrono>
#include <thread>
#include <iostream>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Name.hh>

class Contacts : public gz::sim::System, public gz::sim::ISystemPreUpdate,
                 public gz::sim::ISystemPostUpdate {
 bool initialized=false;
 double previous=-1;
 public:
 void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &ecm) override {
  if(initialized) return;
  std::vector<gz::sim::Entity> ids;
  ecm.Each<gz::sim::components::Collision>([&](const auto &id, auto *){ids.push_back(id);return true;});
  for(auto id:ids) ecm.CreateComponent(id,gz::sim::components::ContactSensorData());
  initialized=true;
 }
 void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override {
  double t=std::chrono::duration<double>(info.simTime).count();
  if(t-previous<.5) return;
  previous=t;
  ecm.Each<gz::sim::components::Collision,gz::sim::components::Name,gz::sim::components::ContactSensorData>(
   [&](const auto &, const auto *, const auto *name, const auto *data){
    if(name->Data()=="collision") return true; // skip field's duplicate side
    for(const auto &c:data->Data().contact()) {
     for(int i=0;i<c.position_size();++i) {
      const auto &p=c.position(i);
      std::cerr<<"contact,"<<t<<","<<name->Data()<<","<<p.x()<<","<<p.y()<<","<<p.z();
      if(i<c.normal_size()) {const auto &n=c.normal(i);std::cerr<<","<<n.x()<<","<<n.y()<<","<<n.z();}
      std::cerr<<std::endl;
     }
    }
    return true;
   });
 }
};

std::mutex mutex;
std::atomic<double> sim_time{0};
double last_print=-1;
void pose(const ignition::msgs::Pose_V &msg) {
  std::lock_guard<std::mutex> guard(mutex);
  for (const auto &p: msg.pose()) {
    if(p.name()!="robot") continue;
    double t=p.header().stamp().sec()+p.header().stamp().nsec()*1e-9;
    sim_time=t;
    if(t-last_print<.2) continue;
    last_print=t;
    const auto &q=p.orientation();
    double pitch=std::asin(std::clamp(2*(q.w()*q.y()-q.z()*q.x()),-1.,1.));
    double roll=std::atan2(2*(q.w()*q.x()+q.y()*q.z()),1-2*(q.x()*q.x()+q.y()*q.y()));
    std::cout<<t<<","<<p.position().x()<<","<<p.position().y()<<","<<p.position().z()<<","<<roll<<","<<pitch<<std::endl;
  }
}
int main(int argc,char **argv){
 if(argc!=5) return 2;
 gz::sim::ServerConfig config;
 config.SetSdfFile(argv[1]);
 gz::sim::Server server(config);
 server.AddSystem(std::make_shared<Contacts>());
 ignition::transport::Node node;
 node.Subscribe("/model/robot/pose_static",pose);
 auto pub=node.Advertise<ignition::msgs::Twist>("/model/robot/cmd_vel");
 double vx=std::stod(argv[2]), vy=std::stod(argv[3]), duration=std::stod(argv[4]);
 std::cout<<"sim_time,x,y,z,roll_rad,pitch_rad"<<std::endl;
 server.Run(false,0,false);
 auto start=std::chrono::steady_clock::now();
 while(sim_time<duration+3 && std::chrono::steady_clock::now()-start<std::chrono::seconds(45)) {
  ignition::msgs::Twist cmd;
  bool driving=sim_time>=1 && sim_time<duration+1;
  cmd.mutable_linear()->set_x(driving?vx:0);
  cmd.mutable_linear()->set_y(driving?vy:0);
  pub.Publish(cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
 }
 server.Stop();
 return sim_time>=duration+3 ? 0 : 3;
}
