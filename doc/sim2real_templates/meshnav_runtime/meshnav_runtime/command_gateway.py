"""Reference ROS gateway; every launch starts disabled and requires explicit enable.

Boolean heartbeats prove recent receipt only. Drivers/localization must publish
truthful state, and the motor controller must stop independently on link loss.
TF is checked at the last valid odometry measurement time; this module does not
detect localization jumps or certify raw sensor data quality.
"""

import json
import math
import time

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

from meshnav_runtime.motion_gate import Limits, MotionGate, ZERO


def _finite(values):
    return all(math.isfinite(value) for value in values)


def _stamp_seconds(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return float("nan")
    return stamp.sec + stamp.nanosec * 1e-9


def _quaternion_valid(q):
    values = (q.x, q.y, q.z, q.w)
    return _finite(values) and abs(sum(value * value for value in values) - 1.0) <= 0.01


def _cloud_layout_valid(message):
    """Validate XYZ layout, including a legitimate empty processed cloud.

    Processing nodes retain responsibility for rejecting nonfinite individual
    points. Reading all raw XYZ values here would duplicate point-cloud filtering
    and is deliberately outside this gateway's bounded callback work.
    """
    if message.height <= 0 or message.point_step <= 0:
        return False
    fields = {field.name: field for field in message.fields}
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in (PointField.FLOAT32, PointField.FLOAT64):
            return False
        size = 4 if field.datatype == PointField.FLOAT32 else 8
        if field.offset < 0 or field.offset + size > message.point_step:
            return False
    return (message.row_step >= message.width * message.point_step
            and len(message.data) == message.row_step * message.height)


class CommandGateway(Node):
    def __init__(self):
        super().__init__("command_gateway")
        defaults = {
            "input_topic": "/nav/cmd_vel", "output_topic": "/hardware/cmd_vel",
            "odom_topic": "/odom", "cloud_topic": "/cloud",
            "obstacle_topic": "/dynamic_obstacles",
            "hardware_ready_topic": "/hardware/ready", "estop_topic": "/hardware/estop",
            "localization_health_topic": "/localization/healthy",
            "map_frame": "map", "odom_frame": "odom",
            "base_frame": "base_footprint", "lidar_frame": "lidar_link",
            "odom_qos": "reliable", "cloud_qos": "best_effort", "obstacle_qos": "reliable",
            "publish_frequency_hz": 20.0,
        }
        self.params = {name: self.declare_parameter(name, value).value
                       for name, value in defaults.items()}
        limit_params = {name: self.declare_parameter(name, value).value
                        for name, value in vars(Limits()).items()}
        self.gate = MotionGate(Limits(**limit_params), self.params["base_frame"])
        if (self.get_parameter("use_sim_time").value
                and not self.params["output_topic"].startswith("/test/")):
            raise ValueError("Sim-time replay requires an isolated /test/ output; hardware deployment uses use_sim_time=false")
        rate = self.params["publish_frequency_hz"]
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("publish_frequency_hz must be finite and positive")
        for name in ("map_frame", "odom_frame", "base_frame", "lidar_frame"):
            value = self.params[name]
            if not isinstance(value, str) or not value or value.startswith("/"):
                raise ValueError(f"{name} must be a nonempty TF frame without a leading slash")
        if len({self.params[name] for name in ("map_frame", "odom_frame", "base_frame", "lidar_frame")}) != 4:
            raise ValueError("map, odom, base and lidar frames must be distinct")
        if self.params["input_topic"] == self.params["output_topic"]:
            raise ValueError("Input and output topics must differ")
        self.output = self.create_publisher(Twist, self.params["output_topic"], 1)
        self.status_output = self.create_publisher(
            String, "~/status", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._odom_stamp = None
        self._last_status = None
        self.create_subscription(TwistStamped, self.params["input_topic"], self._command, 1)
        self.create_subscription(Odometry, self.params["odom_topic"], self._odom,
                                 self._qos(self.params["odom_qos"]))
        self.create_subscription(PointCloud2, self.params["cloud_topic"],
                                 lambda msg: self._cloud("cloud", msg), self._qos(self.params["cloud_qos"]))
        self.create_subscription(PointCloud2, self.params["obstacle_topic"],
                                 lambda msg: self._cloud("obstacles", msg), self._qos(self.params["obstacle_qos"]))
        for topic_name, health_name in (("hardware_ready_topic", "hardware_ready"),
                                        ("estop_topic", "estop"),
                                        ("localization_health_topic", "localization")):
            self.create_subscription(Bool, self.params[topic_name],
                                     lambda msg, key=health_name: self._health(key, msg), 1)
        self.create_service(SetBool, "~/enable", self._enable)
        self.create_timer(1.0 / rate, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.add_on_set_parameters_callback(lambda params: SetParametersResult(
            successful=False, reason="Gateway settings are fixed at startup; disable and restart to change them"))
        self._publish(ZERO)
        self._publish_status()
        self.get_logger().info("Command gateway started disabled; enable only after checking hardware and all input health")

    @staticmethod
    def _qos(value):
        if value not in ("reliable", "best_effort"):
            raise ValueError("QoS reliability must be reliable or best_effort")
        return QoSProfile(depth=1, reliability=(ReliabilityPolicy.RELIABLE if value == "reliable"
                                              else ReliabilityPolicy.BEST_EFFORT))

    def _now(self):
        return time.monotonic(), self.get_clock().now().nanoseconds * 1e-9

    def _publish(self, velocity):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = velocity
        self.output.publish(message)

    def _publish_fault_if_disabled(self):
        if not self.gate.enabled:
            self._publish(ZERO)
            self._publish_status()

    def _command(self, msg):
        now, ros_now = self._now()
        twist = msg.twist
        # All six values are checked; nonplanar inputs are never silently dropped.
        if not _finite((twist.linear.x, twist.linear.y, twist.linear.z,
                        twist.angular.x, twist.angular.y, twist.angular.z)):
            self.gate.reject("command:nonfinite_velocity")
        elif any(abs(value) > 1e-9 for value in (twist.linear.z, twist.angular.x, twist.angular.y)):
            self.gate.reject("command:nonplanar_velocity")
        else:
            self.gate.observe_command((twist.linear.x, twist.linear.y, twist.angular.z),
                                      msg.header.frame_id, _stamp_seconds(msg.header.stamp), now, ros_now)
        self._publish_fault_if_disabled()

    def _odom(self, msg):
        now, ros_now = self._now()
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        valid = (msg.header.frame_id == self.params["odom_frame"]
                 and msg.child_frame_id == self.params["base_frame"]
                 and _quaternion_valid(q)
                 and _finite((p.x, p.y, p.z, t.linear.x, t.linear.y, t.linear.z,
                              t.angular.x, t.angular.y, t.angular.z)))
        self.gate.observe_data("odom", _stamp_seconds(msg.header.stamp), now, ros_now, valid)
        if "odom" in self.gate._data:
            self._odom_stamp = msg.header.stamp
        else:
            self._odom_stamp = None
        self._publish_fault_if_disabled()

    def _cloud(self, key, msg):
        now, ros_now = self._now()
        expected = self.params["lidar_frame"] if key == "cloud" else self.params["base_frame"]
        valid = msg.header.frame_id == expected and _cloud_layout_valid(msg)
        self.gate.observe_data(key, _stamp_seconds(msg.header.stamp), now, ros_now, valid)
        self._publish_fault_if_disabled()

    def _health(self, key, msg):
        now, _ = self._now()
        self.gate.observe_health(key, msg.data, now)
        self._publish_fault_if_disabled()

    def _check_tf(self, now, ros_now):
        if self._odom_stamp is None:
            self.gate.observe_tf(0.0, now, ros_now, False)
            return
        try:
            stamp = Time.from_msg(self._odom_stamp, clock_type=self.get_clock().clock_type)
            for target, source in ((self.params["map_frame"], self.params["odom_frame"]),
                                   (self.params["odom_frame"], self.params["base_frame"])):
                transform = self.tf_buffer.lookup_transform(target, source, stamp)
                translation = transform.transform.translation
                if (not _finite((translation.x, translation.y, translation.z))
                        or not _quaternion_valid(transform.transform.rotation)):
                    raise ValueError("TF contains an invalid translation or quaternion")
            self.gate.observe_tf(_stamp_seconds(self._odom_stamp), now, ros_now)
        except Exception as error:
            self.gate.observe_tf(_stamp_seconds(self._odom_stamp), now, ros_now, False)
            self.get_logger().warn(f"Motion disabled: TF check failed: {error}", throttle_duration_sec=2.0)

    def _enable(self, request, response):
        now, ros_now = self._now()
        if request.data:
            self._check_tf(now, ros_now)
        response.success, response.message = self.gate.set_enabled(request.data, now, ros_now)
        self._publish(ZERO)
        self._publish_status()
        return response

    def _tick(self):
        now, ros_now = self._now()
        self._check_tf(now, ros_now)
        self._publish(self.gate.tick(now, ros_now))
        self._publish_status()

    def _publish_status(self):
        now, ros_now = self._now()
        status = json.dumps(self.gate.status(now, ros_now), ensure_ascii=False, sort_keys=True)
        self.status_output.publish(String(data=status))
        if status != self._last_status:
            self.get_logger().info(status)
            self._last_status = status

    def destroy_node(self):
        if rclpy.ok():
            self._publish(ZERO)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CommandGateway()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
