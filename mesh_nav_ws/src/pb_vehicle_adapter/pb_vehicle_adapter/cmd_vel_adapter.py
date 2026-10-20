"""单路 MeshNav 速度接口：TwistStamped -> Twist，指令失效时持续输出零速度。

只转换消息格式并检查指令时效，不做坐标变换、运动学转换或位置保持。
所有接口参数在启动时读取，修改配置后重启节点。
"""

import math
import time

from geometry_msgs.msg import Twist, TwistStamped
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CmdVelAdapter(Node):
    """检查单路导航指令，向底盘周期发布速度。"""

    def __init__(self, **kwargs):
        super().__init__("pb_cmd_vel_adapter", **kwargs)
        defaults = {
            "input_topic": "/cmd_vel",
            "output_topic": "/base/cmd_vel",
            "base_frame": "base_footprint",
            "timeout_s": 0.5,
            "max_stamp_age_s": 0.5,
            "future_tolerance_s": 0.05,
            "publish_frequency_hz": 30.0,
        }
        self.cfg = {
            name: self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)
            ).value
            for name, default in defaults.items()
        }
        if self.get_parameter("use_sim_time").value:
            raise ValueError("Real-robot velocity adapter requires use_sim_time=false")
        for name in ("timeout_s", "max_stamp_age_s", "publish_frequency_hz"):
            value = float(self.cfg[name])
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        tolerance = float(self.cfg["future_tolerance_s"])
        if not math.isfinite(tolerance) or tolerance < 0:
            raise ValueError("future_tolerance_s must be finite and nonnegative")
        for name in ("input_topic", "output_topic", "base_frame"):
            if not isinstance(self.cfg[name], str) or not self.cfg[name].strip():
                raise ValueError(f"{name} must be a nonempty string")
        self._base_frame = self.cfg["base_frame"].strip().lstrip("/")
        if not self._base_frame:
            raise ValueError("base_frame must name a coordinate frame")
        # 同一话题不能同时承载 TwistStamped 和 Twist；检查也覆盖 ROS remapping。
        if self.resolve_topic_name(self.cfg["input_topic"]) == self.resolve_topic_name(
                self.cfg["output_topic"]):
            raise ValueError("input_topic and output_topic must resolve to different topics")

        self._last_command = None
        self._last_received = None
        self._last_stamp_ns = None
        self._output = self.create_publisher(Twist, self.cfg["output_topic"], 1)
        self._input = self.create_subscription(
            TwistStamped, self.cfg["input_topic"], self._accept, 1
        )
        # 超时由单调时钟判断，不会因系统时间校准而无限延长。
        self._timer = self.create_timer(
            1.0 / self.cfg["publish_frequency_hz"], self._publish_command,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.get_logger().info(
            f"{self._input.topic_name} (TwistStamped) -> "
            f"{self._output.topic_name} (Twist), frame={self._base_frame}"
        )

    def _clear(self):
        self._last_command = None
        self._last_received = None
        self._last_stamp_ns = None
        self._output.publish(Twist())

    def _reject(self, reason):
        # 不继续重发上一条非零命令；下一条有效指令可以恢复输出。
        self._clear()
        self.get_logger().warn(reason, throttle_duration_sec=2.0)

    def _accept(self, message):
        frame = message.header.frame_id.strip().lstrip("/")
        if frame != self._base_frame:
            self._reject(f"Expected velocity frame '{self._base_frame}', got '{frame}'")
            return
        values = (
            message.twist.linear.x, message.twist.linear.y, message.twist.linear.z,
            message.twist.angular.x, message.twist.angular.y, message.twist.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self._reject("Velocity contains non-finite values")
            return
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            self._reject("Velocity requires a valid nonzero timestamp")
            return
        age = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        if not -self.cfg["future_tolerance_s"] <= age <= self.cfg["max_stamp_age_s"]:
            self._reject("Velocity timestamp is stale or too far in the future")
            return
        if self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns:
            self._reject("Velocity timestamp did not advance")
            return
        self._last_command = message.twist
        self._last_received = time.monotonic()
        self._last_stamp_ns = stamp_ns

    def _publish_command(self):
        if self._last_command is None:
            self._output.publish(Twist())
            return
        age = (self.get_clock().now().nanoseconds - self._last_stamp_ns) * 1e-9
        # 每次输出同时检查接收间隔和原始测量时间；重发不会刷新指令有效期。
        if (time.monotonic() - self._last_received > self.cfg["timeout_s"]
                or not -self.cfg["future_tolerance_s"] <= age <= self.cfg["max_stamp_age_s"]):
            self._clear()
            return
        self._output.publish(self._last_command)

    def destroy_node(self):
        # 正常关闭时尽力发送零速度；进程被强制杀死的情况由底盘自身超时处理。
        if rclpy.ok(context=self.context):
            self._clear()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CmdVelAdapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
