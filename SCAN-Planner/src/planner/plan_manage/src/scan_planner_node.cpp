// 启动 SCANReplanFSM

#include <memory>
#include <exception>

#include <rclcpp/rclcpp.hpp>
#include <plan_manage/scan_replan_fsm.h>  // 引入 SCAN 重规划状态机，具体规划流程由它组织。

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("scan_planner_node");

  try  // 捕获下面初始化和回调执行期间传播出的标准异常；上面的 ROS 初始化与节点创建不在此保护范围内。
  {
    scan_planner::SCANReplanFSM planner;  // 创建状态机对象，负责目标输入、局部规划、执行状态和碰撞检查的协调。
    planner.init(node.get()); // 手动调用SCANReplanFSM中的init函数，SCANReplanFSM的构造函数是一个空实现
    rclcpp::executors::SingleThreadedExecutor executor;  // 创建单线程执行器：本执行器调度的回调依次执行，耗时规划会推迟其他回调。
    executor.add_node(node);  // 将节点加入执行器，使其订阅、定时器等回调可以被调度。
    executor.spin();  // 阻塞处理就绪回调，直至上下文关闭或执行器被取消；这里不是直接调用一次规划函数。
  } 
  catch (const std::exception &error)  // 按常量引用捕获标准异常，避免复制并保留异常的动态类型。
  {  // 异常处理分支开始。
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize SCAN-Planner: %s", error.what());  // 输出致命级日志；文字写的是初始化失败，但这里也可能捕获运行期异常，日志本身不会终止进程。
    rclcpp::shutdown();  // 关闭 ROS 2 默认上下文，停止依赖该上下文的 ROS 活动。
    return 1;  // 以非零退出码结束程序，向调用方报告失败；main 中的 node 随退出释放。
  }

  rclcpp::shutdown();
  return 0;
}
