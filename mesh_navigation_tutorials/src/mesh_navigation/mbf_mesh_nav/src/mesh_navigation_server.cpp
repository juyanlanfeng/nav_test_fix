/*
 *  Copyright 2020, Sebastian Pütz
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#include "mbf_mesh_nav/mesh_navigation_server.h"

#include <algorithm>
#include <functional>
#include <cmath>
#include <limits>

#include <geometry_msgs/msg/pose_array.hpp>
#include <mesh_map/mesh_map.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/logging.hpp>

namespace mbf_mesh_nav
{
using namespace std::placeholders;

MeshNavigationServer::MeshNavigationServer(const TFPtr& tf_listener_ptr, const rclcpp::Node::SharedPtr& node)
  : SimpleNavigationServer(tf_listener_ptr, node, false)
  , recovery_plugin_loader_("mbf_mesh_core", "mbf_mesh_core::MeshRecovery")
  , controller_plugin_loader_("mbf_mesh_core", "mbf_mesh_core::MeshController")
  , planner_plugin_loader_("mbf_mesh_core", "mbf_mesh_core::MeshPlanner")
  , mesh_ptr_(new mesh_map::MeshMap(*tf_listener_ptr_, node))
{
  node_->declare_parameter("path_check_step", 0.025);
  node_->declare_parameter("path_check_max_distance", 0.4);
  // advertise services and current goal topic
  check_pose_cost_srv_ =
      node_->create_service<mbf_msgs::srv::CheckPose>("~/check_pose_cost", std::bind(&MeshNavigationServer::callServiceCheckPoseCost, this, _1, _2, _3));
  check_path_cost_srv_ =
      node_->create_service<mbf_msgs::srv::CheckPath>("~/check_path_cost", std::bind(&MeshNavigationServer::callServiceCheckPathCost, this, _1, _2, _3));
  clear_mesh_srv_ = node_->create_service<std_srvs::srv::Empty>("~/clear_mesh", std::bind(&MeshNavigationServer::callServiceClearMesh, this, _1, _2, _3));

  RCLCPP_INFO_STREAM(node_->get_logger(), "Reading map file...");
  mesh_ptr_->readMap();

  // initialize all plugins (also done by SimpleNavigationServer constructor) and then initialize the server components (e.g. services) that depend on the plugins to be loaded and initialized
  initializeServerComponents();
}

mbf_abstract_nav::AbstractPlannerExecution::Ptr MeshNavigationServer::newPlannerExecution(
    const std::string &plugin_name, const mbf_abstract_core::AbstractPlanner::Ptr plugin_ptr)
{
  return std::make_shared<mbf_mesh_nav::MeshPlannerExecution>(
      plugin_name, std::static_pointer_cast<mbf_mesh_core::MeshPlanner>(plugin_ptr), robot_info_, mesh_ptr_, node_);
}

mbf_abstract_nav::AbstractControllerExecution::Ptr MeshNavigationServer::newControllerExecution(
    const std::string &plugin_name, const mbf_abstract_core::AbstractController::Ptr plugin_ptr)
{
  return std::make_shared<mbf_mesh_nav::MeshControllerExecution>(
      plugin_name, std::static_pointer_cast<mbf_mesh_core::MeshController>(plugin_ptr), robot_info_,
      vel_pub_, goal_pub_,
      mesh_ptr_, node_);
}

mbf_abstract_nav::AbstractRecoveryExecution::Ptr MeshNavigationServer::newRecoveryExecution(
    const std::string &plugin_name, const mbf_abstract_core::AbstractRecovery::Ptr plugin_ptr)
{
  return std::make_shared<mbf_mesh_nav::MeshRecoveryExecution>(
      plugin_name, std::static_pointer_cast<mbf_mesh_core::MeshRecovery>(plugin_ptr), robot_info_,
      mesh_ptr_, node_);
}

mbf_abstract_core::AbstractPlanner::Ptr MeshNavigationServer::loadPlannerPlugin(const std::string& planner_type)
{
  mbf_abstract_core::AbstractPlanner::Ptr planner_ptr;
  RCLCPP_INFO(node_->get_logger(), "[MeshNavigationServer] Load global planner plugin.");
  try
  {
    planner_ptr = this->planner_plugin_loader_.createSharedInstance(planner_type);
  }
  catch (const pluginlib::PluginlibException &ex)
  {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "[MeshNavigationServer] Failed to load the " << planner_type << " planner, are you sure it is properly registered"
      << " and that the containing library is built? Exception: " << ex.what());
  }

  if(planner_ptr)
  {
    RCLCPP_INFO(node_->get_logger(), "[MeshNavigationServer] Global planner plugin loaded.");
  }
  
  return planner_ptr;
}

bool MeshNavigationServer::initializePlannerPlugin(
  const std::string& name,
  const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[MeshNavigationServer] Initialize planner \"" << name << "\".");

  mbf_mesh_core::MeshPlanner::Ptr mesh_planner_ptr =
      std::dynamic_pointer_cast<mbf_mesh_core::MeshPlanner>(planner_ptr);
  if (mesh_planner_ptr)
  {
    if (!mesh_ptr_)
    {
      RCLCPP_FATAL_STREAM(node_->get_logger(), "[MeshNavigationServer] The mesh pointer has not been initialized!");
      return false;
    }
    if(mesh_planner_ptr->initialize(name, mesh_ptr_, node_))
    {
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "[MeshNavigationServer] Planner plugin \"" << name << "\" initialized.");
      return true;
    } else {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "[MeshNavigationServer] Failed to initialize plugin " << name << ". The plugin's initialize method returned false, which indicates that the plugin failed to initialize itself properly.");
      return false;
    }
  }

  RCLCPP_ERROR_STREAM(node_->get_logger(), "[MeshNavigationServer] Failed to initialize plugin " << name << ". Looks like it is neither a mesh planner nor a simple planner.");
  return false;
}

mbf_abstract_core::AbstractController::Ptr
MeshNavigationServer::loadControllerPlugin(const std::string& controller_type)
{
  mbf_abstract_core::AbstractController::Ptr controller_ptr;

  RCLCPP_INFO(node_->get_logger(), "[MeshNavigationServer] Load controller plugin.");
  try
  {
    controller_ptr = this->controller_plugin_loader_.createSharedInstance(controller_type);
  }
  catch (const pluginlib::PluginlibException &ex)
  {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "[MeshNavigationServer] Failed to load the " << controller_type << " controller, are you sure it is properly registered"
    << " and that the containing library is built? Exception: " << ex.what());
  }

  if(controller_ptr)
  {
    RCLCPP_INFO(node_->get_logger(), "[MeshNavigationServer] Controller plugin loaded.");
  }

  return controller_ptr;
}

bool MeshNavigationServer::initializeControllerPlugin(
  const std::string& name,
  const mbf_abstract_core::AbstractController::Ptr& controller_ptr)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Initialize controller \"" << name << "\".");

  if (!tf_listener_ptr_)
  {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "The tf listener pointer has not been initialized!");
    return false;
  }

  mbf_mesh_core::MeshController::Ptr mesh_controller_ptr =
      std::dynamic_pointer_cast<mbf_mesh_core::MeshController>(controller_ptr);
  if (mesh_controller_ptr) 
  {
    if (!mesh_ptr_)
    {
      RCLCPP_FATAL_STREAM(node_->get_logger(), "The mesh pointer has not been initialized!");
      return false;
    }
    
    if(mesh_controller_ptr->initialize(name, tf_listener_ptr_, mesh_ptr_, node_))
    {
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Controller plugin \"" << name << "\" initialized.");
      return true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to initialize plugin " << name << ". The plugin's initialize method returned false, which indicates that the plugin failed to initialize itself properly.");
      return false;
    }
  }

  RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to initialize plugin " << name << ". Looks like it is neither a mesh controller nor a simple controller.");
  return false;
}

mbf_abstract_core::AbstractRecovery::Ptr MeshNavigationServer::loadRecoveryPlugin(const std::string& recovery_type)
{
  // return loadPlugin<mbf_abstract_core::AbstractRecovery>(recovery_type, recovery_plugin_loader_, simple_recovery_plugin_loader_, "recovery behavior", node_->get_logger());
  mbf_abstract_core::AbstractRecovery::Ptr recovery_ptr;

  RCLCPP_INFO(node_->get_logger(), "[MeshNavigationServer] Load recovery behavior plugin.");
  try
  {
    recovery_ptr = this->recovery_plugin_loader_.createSharedInstance(recovery_type);
  }
  catch (const pluginlib::PluginlibException &ex)
  {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "[MeshNavigationServer] Failed to load the " << recovery_type << " recovery behavior, are you sure it is properly registered"
      << " and that the containing library is built? Exception: " << ex.what());
  }

  if(recovery_ptr)
  {
    RCLCPP_INFO(node_->get_logger(), "[MeshNavigationServer] Recovery behavior plugin loaded.");
  }

  return recovery_ptr;
}

bool MeshNavigationServer::initializeRecoveryPlugin(
  const std::string& name,
  const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Initialize recovery behavior \"" << name << "\".");

  if (!tf_listener_ptr_)
  {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "The tf listener pointer has not been initialized!");
    return false;
  }

  mbf_mesh_core::MeshRecovery::Ptr mesh_behavior_ptr =
      std::dynamic_pointer_cast<mbf_mesh_core::MeshRecovery>(behavior_ptr);
  if (mesh_behavior_ptr) 
  {
    if (!mesh_ptr_)
    {
      RCLCPP_FATAL_STREAM(node_->get_logger(), "The mesh pointer has not been initialized!");
      return false;
    }

    if(mesh_behavior_ptr->initialize(name, tf_listener_ptr_, mesh_ptr_, node_))
    {
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Recovery behavior plugin \"" << name << "\" initialized.");
      return true;
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to initialize plugin " << name << ". The plugin's initialize method returned false, which indicates that the plugin failed to initialize itself properly.");
      return false;
    }
  }

  RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to initialize plugin " << name << ". Looks like it is neither a mesh recovery behavior nor a simple recovery behavior.");
  return false;
}

void MeshNavigationServer::stop()
{
  Base::stop();
  // TODO
  // RCLCPP_INFO_STREAM_NAMED(node_->get_logger(), "mbf_mesh_nav", "Stopping mesh map for shutdown");
  // mesh_ptr_->stop();
}

MeshNavigationServer::~MeshNavigationServer()
{
  // Mesh plugins are created by the loaders declared in this derived class,
  // while the actions and plugin managers retaining them live in the base
  // class. Derived members are destroyed before the base destructor runs, so
  // waiting for SimpleNavigationServer::~SimpleNavigationServer() would try
  // to unload these libraries while plugin instances still exist. Release all
  // retained instances here, before the mesh class loaders are destroyed.
  planner_action_.reset();
  plan_refiner_action_.reset();
  controller_action_.reset();
  recovery_action_.reset();
  planner_plugin_manager_.clearPlugins();
  plan_refiner_plugin_manager_.clearPlugins();
  controller_plugin_manager_.clearPlugins();
  recovery_plugin_manager_.clearPlugins();
}

void MeshNavigationServer::callServiceCheckPoseCost(std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<mbf_msgs::srv::CheckPose::Request> request, std::shared_ptr<mbf_msgs::srv::CheckPose::Response> response)
{
  // TODO implement
}

void MeshNavigationServer::callServiceCheckPathCost(std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<mbf_msgs::srv::CheckPath::Request> request, std::shared_ptr<mbf_msgs::srv::CheckPath::Response> response)
{
  using Response = mbf_msgs::srv::CheckPath::Response;
  response->state = Response::UNKNOWN;
  // Inflation already encodes vehicle clearance. Footprint checks are unsupported.
  if (!request->path_cells_only || request->costmap != request->GLOBAL_COSTMAP ||
      request->skip_poses != 0 || request->path.poses.empty()) return;
  try
  {
    const double step = node_->get_parameter("path_check_step").as_double();
    const double max_distance = node_->get_parameter("path_check_max_distance").as_double();
    const auto planners = node_->get_parameter("planners").as_string_array();
    if (planners.empty()) return;
    const double limit = node_->get_parameter(planners.front() + ".cost_limit").as_double();
    if (!std::isfinite(step) || step <= 0 || !std::isfinite(max_distance) ||
        max_distance <= 0 || !std::isfinite(limit) || limit <= 0) return;
    std::vector<mesh_map::Vector> points;
    for (auto pose : request->path.poses)
    {
      if (pose.header.frame_id.empty()) pose.header = request->path.header;
      if (pose.header.frame_id.empty()) return;
      const auto p = mesh_ptr_->transformToMapFrame(pose).pose.position;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
      points.emplace_back(p.x, p.y, p.z);
    }
    const auto costs = mesh_ptr_->vertexCostsSnapshot();
    if (!mesh_ptr_->mesh() || !mesh_ptr_->closestPointQueryInterface()) return;
    double total_cost = 0;
    for (size_t i = 0; i < points.size(); ++i)
    {
      response->last_checked = i;
      const auto start = points[i == 0 ? 0 : i - 1];
      const auto delta = points[i] - start;
      const double length = delta.length();
      // Bound malformed requests rather than blocking the executor indefinitely.
      if (!std::isfinite(length) || length / step > 100000) return;
      const size_t samples = std::max<size_t>(1, std::ceil(length / step));
      for (size_t j = (i == 0 ? 0 : 1); j <= samples; ++j)
      {
        auto position = start + delta * (static_cast<float>(j) / samples);
        const lvr2::Vector3f query(position.x, position.y, position.z);
        const auto closest = mesh_ptr_->closestPointQueryInterface()->getClosestPoint(query);
        if (!closest) return;
        if ((closest->point - query).norm() > max_distance)
        { response->state = Response::OUTSIDE; return; }
        const auto vertices = mesh_ptr_->mesh()->getVerticesOfFace(closest->face);
        const auto triangle = mesh_ptr_->mesh()->getVertexPositionsOfFace(closest->face);
        const auto as_double = [](const mesh_map::Vector& v) {
          return Eigen::Vector3d(v.x, v.y, v.z);
        };
        const Eigen::Vector3d a = as_double(triangle[0]);
        const Eigen::Vector3d ab = as_double(triangle[1]) - a;
        const Eigen::Vector3d ac = as_double(triangle[2]) - a;
        const Eigen::Vector3d ap = closest->point.cast<double>() - a;
        const double denominator = ab.squaredNorm() * ac.squaredNorm() - std::pow(ab.dot(ac), 2);
        if (denominator <= 1e-20) return;  // Degenerate triangle: unknown, never free.
        const double v = (ac.squaredNorm() * ap.dot(ab) - ab.dot(ac) * ap.dot(ac)) / denominator;
        const double w = (ab.squaredNorm() * ap.dot(ac) - ab.dot(ac) * ap.dot(ab)) / denominator;
        std::array<double, 3> weights{1 - v - w, v, w};
        double sum = 0;
        for (auto& weight : weights)
        {
          // The closest point lies on this triangle; allow float rounding at edges.
          if (!std::isfinite(weight) || weight < -1e-4 || weight > 1 + 1e-4) return;
          weight = std::clamp(weight, 0.0, 1.0);
          sum += weight;
        }
        for (auto& weight : weights) weight /= sum;
        double cost = 0;
        for (size_t k = 0; k < 3; ++k)
        {
          if (weights[k] <= 1e-6f) continue;
          const auto value = costs.get(vertices[k]);
          if (!value || std::isnan(*value)) return;
          cost += weights[k] * *value;
        }
        if (cost >= limit) { response->state = Response::LETHAL; return; }
        if (!std::isfinite(cost)) return;
        total_cost += std::max(0.0, cost);
      }
    }
    response->cost = static_cast<uint32_t>(std::min(
        total_cost, static_cast<double>(std::numeric_limits<uint32_t>::max())));
    response->state = Response::FREE;
  }
  catch (const std::exception& error)
  {
    RCLCPP_WARN(node_->get_logger(), "Path feasibility check failed: %s", error.what());
  }
}

void MeshNavigationServer::callServiceClearMesh(std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  mesh_ptr_->resetLayers();
}

} /* namespace mbf_mesh_nav */
