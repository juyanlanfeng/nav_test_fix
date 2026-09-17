"""Unified goal/cancel client for the PB MeshNav / JIE / DDDMR frameworks.

    ros2 run pb_vehicle_adapter pb_nav_goal --framework meshnav --case smoke
    ros2 run pb_vehicle_adapter pb_nav_goal --framework jie --x X --y Y --z Z --yaw YAW
    ros2 run pb_vehicle_adapter pb_nav_goal --framework dddmr --case smoke
    ros2 run pb_vehicle_adapter pb_nav_goal --framework dddmr --cancel

Wraps each framework's native interface so the user does not have to hand-write
three different action/point-topic flows.  Always uses the map frame and sim time.
"""

import argparse
import math
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
import tf2_ros
import yaml


FRAMEWORKS = ("meshnav", "jie", "dddmr")
DEFAULT_RESULT_TIMEOUT = 180.0
# Arrival tolerance per framework: the controller's own goal tolerance, so that
# "arrived" means the controller stopped where it thinks it is done instead of
# the vehicle passing through the tolerance circle on its way.
#   MeshNav `mesh_controller` uses mbf's `dist_tolerance: 0.2`
#   d1_controller uses `goal_position_tolerance: 0.1`
DEFAULT_ARRIVAL_XY_TOLERANCE = 0.30
ARRIVAL_XY_TOLERANCE = {"meshnav": 0.20, "jie": 0.10, "dddmr": 0.20}

# jie_path_node subscribes to /start_point and /goal_point with
# `QoS(1).transient_local().reliable()`; a default (volatile) publisher is
# incompatible and every point is dropped silently, so the planner never answers.
JIE_POINT_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def _yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _load_smoke(framework):
    path = os.path.join(get_package_share_directory("pb_vehicle_adapter"), "config", "smoke_goal.yaml")
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    return data.get(framework, data["default"])


class NavGoal(Node):
    def __init__(self):
        super().__init__("pb_nav_goal")
        # Section 14: every interface works in map frame and simulation time, so
        # goal stamps match the rest of the stack instead of wall-clock time.
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._jie_goal_pub = self.create_publisher(PointStamped, "/goal_point", JIE_POINT_QOS)
        self._jie_start_pub = self.create_publisher(PointStamped, "/start_point", JIE_POINT_QOS)
        self._jie_start_cmd = self.create_publisher(Bool, "/start_navigation", 10)
        self._jie_stop_cmd = self.create_publisher(Bool, "/stop_navigation", 10)
        self._paths = []
        self.result_timeout = DEFAULT_RESULT_TIMEOUT
        self.plan_only = False
        self.path_out = None

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def robot_pose(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                tf = self._tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
                return tf.transform.translation, tf.transform.rotation
            except Exception:  # noqa: BLE001 - keep waiting for the TF
                rclpy.spin_once(self, timeout_sec=0.1)
        return None, None

    def log_pose(self, label):
        """Log a map-frame pose as acceptance evidence."""
        translation, _rotation = self.robot_pose(timeout=5.0)
        if translation is None:
            self.get_logger().warn("%s unavailable (no map -> base_footprint TF)" % label)
            return
        try:
            tf = self._tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            yaw = 2.0 * math.atan2(tf.transform.rotation.z, tf.transform.rotation.w)
        except Exception:  # noqa: BLE001
            yaw = float("nan")
        self.get_logger().info("%s x=%.3f y=%.3f z=%.3f yaw=%.3f rad"
                               % (label, translation.x, translation.y, translation.z, yaw))

    def cancel(self, handle, label):
        future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        self.get_logger().info("%s cancel requested" % label)

    def await_result(self, handle, timeout, label):
        """Return the action result, or None on timeout/abort.

        rclpy resolves the result future to None when the goal ends without a
        result, so callers must not assume a wrapped result object.
        """
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().error("%s did not finish within %.0f s; cancelling" % (label, timeout))
            self.cancel(handle, label)
            return None
        wrapped = future.result()
        if wrapped is None:
            self.get_logger().error("%s ended without a result (aborted or cancelled)" % label)
            return None
        return wrapped.result

    # ---- plan / path helpers --------------------------------------------
    def save_path(self, path):
        """Record a planned path (map frame) so its z profile can be checked.

        The tunnel acceptance has to prove the path stays on the tunnel floor
        (z < 0.10 m) instead of snapping to the roof top at z = 0.424 m, which
        only a recorded path can show.
        """
        poses = list(path.poses)
        if not poses:
            self.get_logger().error("plan has no poses")
            return None
        ys = [pose.pose.position.y for pose in poses]
        zs = [pose.pose.position.z for pose in poses]
        self.get_logger().info("plan: %d poses, z %.4f..%.4f, y %.3f..%.3f"
                               % (len(poses), min(zs), max(zs), min(ys), max(ys)))
        if not self.path_out:
            return (min(zs), max(zs))
        with open(self.path_out, "w", encoding="utf-8") as stream:
            stream.write("x,y,z,yaw\n")
            for pose in poses:
                q = pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                stream.write("%.5f,%.5f,%.5f,%.5f\n"
                             % (pose.pose.position.x, pose.pose.position.y,
                                pose.pose.position.z, yaw))
        self.get_logger().info("plan written to %s" % self.path_out)
        return (min(zs), max(zs))

    # ---- MeshNav: MBF GetPath then ExePath -------------------------------
    def run_meshnav(self, goal, cancel):
        from mbf_msgs.action import ExePath, GetPath
        from rclpy.action import ActionClient

        get_path = ActionClient(self, GetPath, "/move_base_flex/get_path")
        exe_path = ActionClient(self, ExePath, "/move_base_flex/exe_path")
        for name, client in (("get_path", get_path), ("exe_path", exe_path)):
            if not client.wait_for_server(timeout_sec=15.0):
                self.get_logger().error("MeshNav %s action server unavailable" % name)
                return 1
        request = GetPath.Goal()
        request.use_start_pose = False
        request.target_pose = goal
        request.planner = "mesh_planner"
        future = get_path.send_goal_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("MeshNav GetPath goal rejected")
            return 1
        result = self.await_result(handle, self.result_timeout, "MeshNav GetPath")
        if result is None:
            return 1
        self.get_logger().info("MeshNav GetPath outcome=%d message=%s points=%d"
                               % (result.outcome, result.message, len(result.path.poses)))
        if result.outcome != 0 or not result.path.poses:
            self.get_logger().error("MeshNav planning failed")
            return 1
        self.save_path(result.path)
        if self.plan_only:
            return 0
        execute = ExePath.Goal()
        execute.path = result.path
        execute.controller = "mesh_controller"
        future = exe_path.send_goal_async(execute)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("MeshNav ExePath goal rejected")
            return 1
        if cancel:
            self.spin_for(3.0)
            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=10.0)
            self.get_logger().info("MeshNav execution cancelled")
            return 0
        result = self.await_result(handle, self.result_timeout, "MeshNav ExePath")
        if result is None:
            return 1
        self.get_logger().info("MeshNav ExePath outcome=%d message=%s" % (result.outcome, result.message))
        return 0 if result.outcome == 0 else 1

    # ---- JIE: /start_point + /goal_point, then the start command ---------
    def run_jie(self, goal, cancel):
        if cancel:
            self._jie_stop_cmd.publish(Bool(data=True))
            self.get_logger().info("JIE stop command sent")
            return 0
        translation, _ = self.robot_pose()
        if translation is None:
            self.get_logger().error("JIE: no map -> base_footprint TF for the start point")
            return 1
        start = PointStamped()
        start.header.frame_id = "map"
        start.header.stamp = self.get_clock().now().to_msg()
        start.point.x, start.point.y, start.point.z = translation.x, translation.y, translation.z
        self._jie_start_pub.publish(start)
        target = PointStamped()
        target.header.frame_id = "map"
        target.header.stamp = self.get_clock().now().to_msg()
        target.point.x = goal.pose.position.x
        target.point.y = goal.pose.position.y
        target.point.z = goal.pose.position.z
        self._jie_goal_pub.publish(target)
        self.get_logger().info("JIE start=(%.2f, %.2f, %.2f) goal=(%.2f, %.2f, %.2f)"
                               % (start.point.x, start.point.y, start.point.z,
                                  target.point.x, target.point.y, target.point.z))
        paths = []
        self.create_subscription(Path, "/planned_path", lambda msg: paths.append(msg), 10)
        deadline = time.monotonic() + DEFAULT_RESULT_TIMEOUT
        while time.monotonic() < deadline and not paths:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not paths:
            self.get_logger().error("JIE: no /planned_path received")
            return 1
        self.get_logger().info("JIE planned path with %d poses" % len(paths[-1].poses))
        self.save_path(paths[-1])
        if self.plan_only:
            return 0
        self._jie_start_cmd.publish(Bool(data=True))
        self.get_logger().info("JIE start command sent")
        # d1_controller has no arrival topic, so completion is judged from the
        # live map pose, exactly like the manual acceptance procedure, using the
        # controller's own goal tolerance.
        return 0 if self.wait_for_arrival(goal, ARRIVAL_XY_TOLERANCE["jie"]) else 1

    def wait_for_arrival(self, goal, tolerance_xy=DEFAULT_ARRIVAL_XY_TOLERANCE):
        """Wait until the vehicle really stands within `tolerance_xy` of the goal.

        A plain "distance below tolerance" test declares arrival while the vehicle
        is still driving: the first JIE tunnel run reported
        "arrived: 0.299 m from the goal (tolerance 0.30 m)" four seconds after the
        start command, with the vehicle still inside the tunnel.  The tolerance is
        therefore held over several consecutive samples before the run counts as
        arrived, so the reported final pose is the settled one.
        """
        deadline = time.monotonic() + self.result_timeout
        best = None
        settled = 0
        while time.monotonic() < deadline:
            translation, _rotation = self.robot_pose(timeout=1.0)
            if translation is not None:
                distance = math.hypot(translation.x - goal.pose.position.x,
                                      translation.y - goal.pose.position.y)
                best = distance if best is None else min(best, distance)
                settled = settled + 1 if distance <= tolerance_xy else 0
                if settled >= 3:
                    self.get_logger().info(
                        "arrived: %.3f m from the goal (tolerance %.2f m, settled)"
                        % (distance, tolerance_xy))
                    return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error("did not arrive within %.0f s (closest %.3f m, tolerance %.2f m)"
                                % (self.result_timeout,
                                   best if best is not None else float("nan"), tolerance_xy))
        return False

    # ---- DDDMR: /p2p_move_base PToPMoveBase action -----------------------
    def run_dddmr(self, goal, cancel):
        try:
            from dddmr_sys_core.action import PToPMoveBase
        except ImportError:
            self.get_logger().error(
                "dddmr_sys_core not found; source third_party/setup_dddmr_env.sh first")
            return 1
        from rclpy.action import ActionClient

        client = ActionClient(self, PToPMoveBase, "/p2p_move_base")
        if not client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("DDDMR /p2p_move_base action server unavailable")
            return 1
        request = PToPMoveBase.Goal()
        request.target_pose = goal
        request.target_value = 0.0
        future = client.send_goal_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("DDDMR goal rejected")
            return 1
        if cancel:
            self.spin_for(3.0)
            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=10.0)
            self.get_logger().info("DDDMR goal cancelled")
            return 0
        result = self.await_result(handle, self.result_timeout, "DDDMR PToPMoveBase")
        if result is None:
            return 1
        # DDDMR: SUCCESS = 1, not 0.
        self.get_logger().info("DDDMR status=%d result=%s" % (result.status, result.result))
        return 0 if result.status == 1 else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--framework", required=True, choices=FRAMEWORKS)
    parser.add_argument("--case", choices=["smoke"], help="use the verified smoke goal")
    parser.add_argument("--x", type=float)
    parser.add_argument("--y", type=float)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0, help="degrees")
    parser.add_argument("--cancel", action="store_true")
    parser.add_argument(
        "--plan-only", action="store_true",
        help="stop after planning: MeshNav skips ExePath, JIE skips /start_navigation",
    )
    parser.add_argument(
        "--path-out", help="write the planned path (map frame) to this CSV file",
    )
    parser.add_argument(
        "--timeout", type=float, default=DEFAULT_RESULT_TIMEOUT,
        help="wall-clock seconds to wait for the action result (default %.0f)" % DEFAULT_RESULT_TIMEOUT,
    )
    args = parser.parse_args()

    if args.case == "smoke":
        smoke = _load_smoke(args.framework)
        goal_cfg = smoke["goal"]
        args.x, args.y = goal_cfg["x"], goal_cfg["y"]
        args.z = goal_cfg.get("z", 0.0)
        args.yaw = goal_cfg.get("yaw", 0.0)
    if not args.cancel and (args.x is None or args.y is None):
        parser.error("provide --x/--y (or --case smoke, or --cancel)")

    rclpy.init()
    node = NavGoal()
    node.result_timeout = args.timeout
    node.plan_only = args.plan_only
    node.path_out = args.path_out
    try:
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = node.get_clock().now().to_msg()
        if args.x is not None:
            goal.pose.position.x = args.x
            goal.pose.position.y = args.y
            goal.pose.position.z = args.z
        qx, qy, qz, qw = _yaw_quaternion(math.radians(args.yaw))
        goal.pose.orientation.x, goal.pose.orientation.y = qx, qy
        goal.pose.orientation.z, goal.pose.orientation.w = qz, qw

        node.log_pose("start pose")
        if args.framework == "meshnav":
            status = node.run_meshnav(goal, args.cancel)
        elif args.framework == "jie":
            status = node.run_jie(goal, args.cancel)
        else:
            status = node.run_dddmr(goal, args.cancel)
        node.log_pose("final pose")
        return status
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
