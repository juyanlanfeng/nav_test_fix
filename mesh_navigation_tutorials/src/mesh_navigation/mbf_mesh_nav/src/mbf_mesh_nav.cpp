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
 */

#include "mbf_mesh_nav/mesh_navigation_server.h"
#include <chrono>
#include <cstdlib>
#include <mbf_utility/types.h>
#include <signal.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

volatile sig_atomic_t shutdown_requested = 0;

void sigintHandler(int)
{
  // A POSIX signal handler may only perform async-signal-safe work.  In
  // particular, neither rclcpp::shutdown() nor action cancellation is safe
  // here.  A wall timer below observes this flag from executor context.
  shutdown_requested = 1;
}

int main(int argc, char** argv)
{
  // Own the ROS context explicitly.  The process-wide default context is only
  // finalized by a static destructor.  With CycloneDDS that allowed its "gc"
  // thread to keep running while librmw_cyclonedds_cpp was being dlclose()'d,
  // causing an intermittent SIGSEGV after an otherwise clean shutdown.
  auto context = std::make_shared<rclcpp::Context>();
  context->init(argc, argv);

  // Keep every ROS entity in this scope.  They must be destroyed while the
  // context is still valid; a global node or an executor surviving
  // rclcpp::shutdown() caused a late process-exit segmentation fault.
  {
    rclcpp::NodeOptions node_options;
    node_options.context(context);
    auto node = std::make_shared<rclcpp::Node>("mbf_mesh_nav", node_options);
    const double cache_time = node->declare_parameter("tf_cache_time", 10.0);

    TFPtr tf_buffer_ptr = std::make_shared<TF>(node->get_clock(), tf2::durationFromSec(cache_time));
    // Bind TF subscriptions to the same explicitly-owned node/context.  The
    // one-argument constructor creates a hidden node on the global context.
    tf2_ros::TransformListener tf_listener(*tf_buffer_ptr, node, false);
    // TF callbacks are serviced by the MultiThreadedExecutor below.  Tell
    // tf2 that another executor thread can continue filling the buffer while
    // an action thread waits in canTransform(..., timeout); otherwise tf2
    // rejects every non-zero-timeout robot-pose lookup immediately.
    tf_buffer_ptr->setUsingDedicatedThread(true);
    RCLCPP_INFO_STREAM(node->get_logger(), "Starting mesh navigation server.");
    auto mesh_nav_srv_ptr = std::make_shared<mbf_mesh_nav::MeshNavigationServer>(tf_buffer_ptr, node);
    signal(SIGINT, sigintHandler);
    signal(SIGTERM, sigintHandler);

    // A multithreaded executor lets cost layers process sensor data in their
    // own callback groups.
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context;
    rclcpp::executors::MultiThreadedExecutor exec(executor_options);
    auto shutdown_timer = node->create_wall_timer(std::chrono::milliseconds(50), [&exec]() {
      if (shutdown_requested)
      {
        exec.cancel();
      }
    });
    exec.add_node(node);
    exec.spin();

    RCLCPP_INFO_STREAM(node->get_logger(), "Stopping mesh navigation server.");

    // Keep the context alive while canceling and joining action execution
    // threads: their final iterations may publish feedback or zero velocity.
    // Destroy plugin instances before pluginlib's process-wide registry.
    mesh_nav_srv_ptr->stop();
    exec.remove_node(node);
    mesh_nav_srv_ptr.reset();
    shutdown_timer.reset();
    RCLCPP_INFO_STREAM(node->get_logger(), "Navigation server stopped cleanly.");
  }

  // Context::~Context performs rcl_context_fini(), which joins the DDS worker
  // threads while their implementation library is still loaded.
  context->shutdown("mesh navigation server stopped");
  context.reset();

  // On ROS 2 Humble with rmw_cyclonedds_cpp, the process-wide RMW loader may
  // dlclose the implementation from a static destructor while CycloneDDS's
  // process-global "gc" thread is still winding down.  GDB confirmed that
  // race as the only remaining post-main SIGSEGV.  All owned resources and
  // the explicit context have already been finalized above, so skip only the
  // unsafe process-global static destructor pass.  The OS then reclaims the
  // process image normally and ros2 launch observes exit status 0.
  std::_Exit(EXIT_SUCCESS);
}
