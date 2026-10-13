"""Start PB simulation and MeshNav from their respective YAML files."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml

def generate_launch_description():

    # [SIM/COMMON] 获取已安装功能包的 share 目录，不是直接使用 src 目录。
    # sim_share、gz_share 是仿真专用；mesh_share 提供可复用的导航配置。
    # 注意：这些查找发生在启动条件判断之前。因此 start_sim=False 仍要求
    # mesh_navigation_tutorials_sim、ros_gz_sim 可被找到，不能当作实车入口。
    pb_share = get_package_share_directory("pb_vehicle_adapter")
    sim_share = get_package_share_directory("mesh_navigation_tutorials_sim")
    mesh_share = get_package_share_directory("mesh_navigation_tutorials")
    gz_share = get_package_share_directory("ros_gz_sim")
    with open(os.path.join(mesh_share, "config", "mbf_mesh_nav.yaml"), encoding="utf-8") as stream:
        nav_config = yaml.safe_load(stream)
    # meshnav_launch 是供本 launch 读取的配置块，不会因此生成同名 ROS 节点。
    # move_base_flex 是导航服务器的参数块；其插件与地图参数由后面的子 launch 加载。
    nav_options = nav_config["meshnav_launch"]["ros__parameters"]
    nav_params = nav_config["move_base_flex"]["ros__parameters"]

    # [COMMON，当前输出接 SIM] 速度适配器选择 control_source 指定的来源。
    # 默认 MeshNav 输入 /cmd_vel 为 TwistStamped，输出 /pb/cmd_vel_safe 为 Twist；
    # 节点负责命令时效检查、周期输出和超时处理，本 launch 没有覆盖这些话题。
    # hold_enabled=True 时，还读取 hold_pose_topic（默认 /odom）及 ExePath 状态，
    # 执行任务结束后的驻车保持；实车是否启用需结合真实里程计与底盘能力决定。
    # [REAL] 可复用或替换为实车速度网关，但要显式配置输入/输出话题、时间、
    # 坐标系和限速策略；输出应交给实际底盘驱动。本节点不实现串口/CAN 通信。
    velocity = Node(
        package="pb_vehicle_adapter",
        executable="pb_cmd_vel_adapter",
        name="pb_cmd_vel_adapter",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "control_source": "meshnav",
            "hold_enabled": True,
            "hold_pose_topic": "/odom",
            "hold_gain": 1.0,
            "hold_max_linear_velocity": 0.1,
            "hold_max_angular_velocity": 0.2,
            "hold_angular_deadband": 0.02,
        }],
    )

    # [COMMON] 启动导航服务器 move_base_flex，由 MBF 子 launch 读取导航 YAML。
    # 同一服务器中加载 MeshMap、地图层、规划器和控制器插件；这些不是各自
    # 单独启动的 ROS 节点。当前配置选择 CVP + MeshController，没有使用 MPPI。
    # 子 launch 将 /move_base_flex/cmd_vel 重映射到 /cmd_vel（TwistStamped）。
    # [REAL] 可复用这些导航包，须提供真实 /odom、定位 TF、PLY 地图和障碍输入，
    # 配置真实车体、运动能力、可写 H5 缓存路径，并将 use_sim_time 设为 false。
    # 地图由 Blender 制作不影响这里对 LVR2 运行时网格库的依赖。
    meshnav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            mesh_share, "launch", "mbf_mesh_navigation_server_launch.py",
        )),
    )
    # [COMMON，可选 UI] RViz 显示地图、路径、点云和 TF，可用于仿真或实车。
    # 此处选择 PB RViz 配置；目标工具的话题需要与 navigator.goal_topic 对应。
    # [REAL] 可在操作电脑上运行，机器人无需打开 GUI；同时调整显示话题和 frame。
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_meshnav",
        output="screen",
        arguments=["-d", os.path.join(pb_share, "rviz", nav_options["meshnav_rviz_config"])],
        parameters=[{"use_sim_time": nav_params["use_sim_time"]}],
        condition=IfCondition(str(nav_options["start_rviz"])),
    )
    # [COMMON] 导航协调节点：默认订阅 /rviz/goal_pose（PoseStamped），
    # 调用 MBF /move_base_flex/get_path（GetPath）和 /exe_path（ExePath），
    # 按 planner_frequency 检查剩余路线代价，受阻时协调重新规划与执行。
    # 同时提供 /meshnav_navigator/status 和 /meshnav_navigator/cancel。
    # planner/controller 是 YAML 中的插件实例名，不是新建节点名。
    # 这里不直接控制电机，不提供定位，也不执行点云分割。
    # [REAL] 可保留；确认目标位姿处于导航地图坐标系，且 action/service 名称一致。
    # 加载整个 YAML 时，按节点名匹配参数块，其他节点的配置不会全部注入该节点。
    navigator = Node(
        package="pb_vehicle_adapter",
        executable="meshnav_navigator",
        name="meshnav_navigator",
        output="screen",
        parameters=[os.path.join(mesh_share, "config", "mbf_mesh_nav.yaml"), {
            "use_sim_time": nav_params["use_sim_time"],
            "planner_frequency": nav_params["planner_frequency"],
            "planner": nav_params["planners"][0],
            "controller": nav_params["controllers"][0],
        }],
    )

    # --wait-timeout 是检查重试超时，不是延后启动导航节点的等待时间。
    # 它检查推进中的 /clock、TF、节点/action 和 PB 速度来源等仿真条件；
    # 只读检查不发送导航目标或速度，也不会作为上面节点的启动屏障。
    # 检查进程退出失败不会自动停止此前启动的节点，本文件未配置这种联动。
    # [REAL] 应换为真实定位、传感器、TF 和底盘接口的检查，不能要求 /clock 推进。
    startup_check = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "run", "pb_vehicle_adapter", "pb_preflight",
                "--framework", "meshnav",
                "--wait-timeout", str(nav_options["startup_timeout_s"]),
            ],
            output="screen",
        )],
    )

    # 启动组织顺序：仿真分组 -> 可选 RMCL -> 既有适配器来源切换 -> MBF
    # -> 导航协调 -> 可选 RViz -> 延迟检查。条件不满足的动作会跳过。
    # 这是 launch 动作的列举顺序；各进程异步运行，不保证按此顺序完成初始化。
    # 真正发起一次导航还需要目标消息，以及已可用的地图、TF、里程计、
    # 所选障碍输入和底盘命令接口；创建这些动作本身不等于接口已就绪。
    return LaunchDescription([
        meshnav,
        navigator,
        rviz,
        startup_check,
        velocity,
    ])
