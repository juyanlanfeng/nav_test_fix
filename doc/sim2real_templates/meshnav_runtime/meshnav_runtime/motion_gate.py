"""ROS-independent explicit-enable gate; a fault always commands zero immediately.

Callers provide monotonic receipt times and ROS measurement times separately.
This gate cannot replace the chassis driver's independent command watchdog.
"""

from dataclasses import dataclass
import math


ZERO = (0.0, 0.0, 0.0)
STAMPED_INPUTS = ("odom", "cloud", "obstacles")
HEALTH_INPUTS = ("hardware_ready", "estop", "localization")


@dataclass(frozen=True)
class Limits:
    command_timeout_s: float = 0.25
    data_timeout_s: float = 0.5
    max_data_age_s: float = 0.30
    health_timeout_s: float = 0.5
    tf_timeout_s: float = 0.5
    max_future_s: float = 0.001
    max_linear_velocity: float = 0.2
    max_angular_velocity: float = 0.3
    max_linear_acceleration: float = 0.5
    max_angular_acceleration: float = 1.0

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")


class MotionGate:
    """Accept fresh planar commands only after explicit enabling with healthy inputs."""

    def __init__(self, limits=None, base_frame="base_footprint"):
        self.limits = limits or Limits()
        if not base_frame or base_frame.startswith("/"):
            raise ValueError("base_frame must be a nonempty TF frame without a leading slash")
        self.base_frame = base_frame
        self.enabled = False
        self.reason = "startup_disabled"
        self._fault = False
        self._data = {}
        self._last_data_stamp = {}
        self._health = {}
        self._tf = None
        self._command = None
        self._command_stamp = None
        self._last_command_stamp = None
        self._enabled_at = None
        self._output = ZERO
        self._tick_time = None
        self._ros_time = None

    def _trip(self, reason):
        self.enabled = False
        self._fault = True
        self.reason = reason
        self._command = None
        self._command_stamp = None
        self._output = ZERO
        return ZERO

    def reject(self, reason):
        """Invalidate an input without refreshing any last-valid receipt time."""
        return self._trip(reason)

    def _stamp_reason(self, stamp, ros_now, age_limit):
        if not math.isfinite(stamp) or stamp <= 0.0:
            return "invalid_stamp"
        if not math.isfinite(ros_now) or ros_now < 0.0:
            return "invalid_ros_time"
        age = ros_now - stamp
        if age < -self.limits.max_future_s:
            return "future_stamp"
        if age > age_limit:
            return "stale_stamp"
        return None

    def observe_data(self, name, stamp, received, ros_now, valid=True):
        if name not in STAMPED_INPUTS:
            raise ValueError(f"Unknown data input: {name}")
        error = self._stamp_reason(stamp, ros_now, self.limits.max_data_age_s)
        previous_stamp = self._last_data_stamp.get(name)
        if not valid:
            error = "invalid_data"
        elif not math.isfinite(received):
            error = "invalid_receipt_time"
        elif error is None and previous_stamp is not None and stamp <= previous_stamp:
            error = "nonincreasing_stamp"
        if error:
            self._data.pop(name, None)
            return self._trip(f"{name}:{error}")
        self._data[name] = (stamp, received)
        self._last_data_stamp[name] = stamp
        return self._output

    def observe_health(self, name, value, received):
        if name not in HEALTH_INPUTS:
            raise ValueError(f"Unknown health input: {name}")
        if not isinstance(value, bool) or not math.isfinite(received):
            return self._trip(f"{name}:invalid_health")
        self._health[name] = (value, received)
        safe = not value if name == "estop" else value
        if not safe:
            return self._trip(f"{name}:unsafe")
        return self._output

    def observe_tf(self, stamp, received, ros_now, valid=True):
        # The wrapper looks up map<-odom<-base at the accepted odometry stamp.
        # Static calibration transforms are allowed; the odometry stamp is fresh.
        error = self._stamp_reason(stamp, ros_now, self.limits.tf_timeout_s)
        if not valid or not math.isfinite(received):
            error = "unavailable_or_invalid"
        if error:
            self._tf = None
            return self._trip(f"tf:{error}")
        self._tf = (stamp, received)
        return self._output

    def readiness_reason(self, now, ros_now):
        if not math.isfinite(now):
            return "invalid_steady_time"
        for name in STAMPED_INPUTS:
            item = self._data.get(name)
            if item is None:
                return f"{name}:missing"
            stamp, received = item
            if not 0.0 <= now - received <= self.limits.data_timeout_s:
                return f"{name}:receipt_timeout"
            error = self._stamp_reason(stamp, ros_now, self.limits.max_data_age_s)
            if error:
                return f"{name}:{error}"
        for name in HEALTH_INPUTS:
            item = self._health.get(name)
            if item is None:
                return f"{name}:missing"
            value, received = item
            if not 0.0 <= now - received <= self.limits.health_timeout_s:
                return f"{name}:receipt_timeout"
            if (value if name == "estop" else not value):
                return f"{name}:unsafe"
        if self._tf is None:
            return "tf:missing"
        stamp, received = self._tf
        if not 0.0 <= now - received <= self.limits.tf_timeout_s:
            return "tf:receipt_timeout"
        error = self._stamp_reason(stamp, ros_now, self.limits.tf_timeout_s)
        return f"tf:{error}" if error else None

    def set_enabled(self, enable, now, ros_now):
        if not enable:
            self.enabled = False
            self._fault = False
            self.reason = "operator_disabled"
            self._command = None
            self._command_stamp = None
            self._output = ZERO
            self._tick_time = now
            return True, self.reason
        reason = self.readiness_reason(now, ros_now)
        if reason:
            self._trip(reason)
            return False, reason
        self.enabled = True
        self._fault = False
        self.reason = "enabled_waiting_for_new_command"
        # Even a repeated enable is a new epoch. Never release a buffered command.
        self._enabled_at = (now, ros_now)
        self._command = None
        self._command_stamp = None
        self._output = ZERO
        self._tick_time = now
        self._ros_time = ros_now
        return True, self.reason

    def observe_command(self, velocity, frame, stamp, received, ros_now):
        error = self._stamp_reason(stamp, ros_now, self.limits.command_timeout_s)
        if frame != self.base_frame:
            error = "frame_mismatch"
        elif len(velocity) != 3 or not all(math.isfinite(v) for v in velocity):
            error = "nonfinite_velocity"
        elif not math.isfinite(received):
            error = "invalid_receipt_time"
        elif error is None and self._last_command_stamp is not None and stamp <= self._last_command_stamp:
            error = "nonincreasing_stamp"
        if error:
            return self._trip(f"command:{error}")
        self._last_command_stamp = stamp
        if not self.enabled:
            return ZERO
        if received < self._enabled_at[0] or stamp <= self._enabled_at[1]:
            return self._trip("command:before_enable")
        self._command = (tuple(velocity), received)
        self._command_stamp = stamp
        self.reason = "enabled"
        return self._output

    def tick(self, now, ros_now):
        if self._ros_time is not None and ros_now < self._ros_time:
            self._ros_time = ros_now
            self._data.clear()
            self._last_data_stamp.clear()
            self._tf = None
            self._last_command_stamp = None
            return self._trip("ros_time_went_backwards")
        self._ros_time = ros_now
        reason = self.readiness_reason(now, ros_now)
        if reason:
            return self._trip(reason)
        if not self.enabled:
            self._output = ZERO
            self._tick_time = now
            return ZERO
        if self._command is None:
            self._tick_time = now
            return ZERO
        command, received = self._command
        if not 0.0 <= now - received <= self.limits.command_timeout_s:
            return self._trip("command:receipt_timeout")
        error = self._stamp_reason(self._command_stamp, ros_now, self.limits.command_timeout_s)
        if error:
            return self._trip(f"command:{error}")
        if command == ZERO:
            self._output = ZERO
            self._tick_time = now
            return ZERO
        dt = max(0.0, now - self._tick_time) if self._tick_time is not None else 0.0
        self._tick_time = now
        vx, vy, wz = command
        norm = math.hypot(vx, vy)
        if norm > self.limits.max_linear_velocity:
            scale = self.limits.max_linear_velocity / norm
            vx, vy = vx * scale, vy * scale
        wz = max(-self.limits.max_angular_velocity, min(self.limits.max_angular_velocity, wz))
        dx, dy = vx - self._output[0], vy - self._output[1]
        change = math.hypot(dx, dy)
        max_change = self.limits.max_linear_acceleration * dt
        if change > max_change:
            dx, dy = dx * max_change / change, dy * max_change / change
        dw = wz - self._output[2]
        max_dw = self.limits.max_angular_acceleration * dt
        dw = max(-max_dw, min(max_dw, dw))
        self._output = (self._output[0] + dx, self._output[1] + dy, self._output[2] + dw)
        return self._output

    def status(self, now, ros_now):
        health_reason = self.readiness_reason(now, ros_now)
        state = "fault" if self._fault else "ready" if health_reason is None else "disabled"
        return dict(state=state, enabled=self.enabled, reason=self.reason,
                    ready=health_reason is None, health_reason=health_reason,
                    waiting_for_command=self.enabled and self._command is None)
