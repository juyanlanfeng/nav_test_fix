"""Post-action station keeping for the PB velocity chain.

Why this exists (doc/PB_SLOPE_NEXT_STEPS_20260921.md P3): the terminal controller
already holds a pose while its ExePath action is alive, but once that action
terminates MBF stops asking for velocity commands and `force_stop_at_goal` publishes
zero. On this vehicle a zero velocity target is not a position brake: the P-only
velocity drive can oppose motion but cannot guarantee zero steady-state drift.
The chassis kept sliding (measured: 0.2649 m over 103 s
after arrival, with `static` 92.9% and `idle` 0.0%, i.e. no commands at all).

A controller *plugin* cannot fix that, because it has no execution opportunity after
its own action ends.  The velocity adapter can: it owns the output topic and already
runs a watchdog, so it is the one place where "the navigation command went away" is
observed and where a corrective command can be published without two publishers
fighting over the same topic.

The logic here is pure and mirrors `pb_terminal_controller`'s
`station_keeping_command` so that the in-action hold and the post-action hold behave
identically.  Nothing here fakes physics: it commands velocity through the same
interface the navigation stack uses.
"""

import math

# The hold is released when a navigation command is accepted; it is re-armed by the
# watchdog on the next timeout.  A hold reference is captured once per arming so the
# correction does not chase the vehicle's own drift.
DEFAULT_GAIN = 1.0
DEFAULT_MAX_LINEAR = 0.1
DEFAULT_MAX_ANGULAR = 0.2
DEFAULT_ANGULAR_DEADBAND = 0.02


def wrap_angle(angle: float) -> float:
    """Wrap an angle into (-pi, pi]."""
    return math.remainder(angle, 2.0 * math.pi)


def station_keeping_command(dx: float, dy: float, yaw_error: float, yaw: float,
                            gain: float = DEFAULT_GAIN,
                            max_linear: float = DEFAULT_MAX_LINEAR,
                            max_angular: float = DEFAULT_MAX_ANGULAR,
                            angular_deadband: float = DEFAULT_ANGULAR_DEADBAND):
    """Bounded body-frame velocity that opposes the measured offset from a held pose.

    `dx`/`dy` are the held pose minus the current pose, expressed in the map frame;
    `yaw` is the current body yaw.  Returns `(linear_x, linear_y, angular_z)`.
    """
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    forward = cos_yaw * dx + sin_yaw * dy
    lateral = -sin_yaw * dx + cos_yaw * dy
    linear_x = max(-max_linear, min(max_linear, gain * forward))
    linear_y = max(-max_linear, min(max_linear, gain * lateral))
    if abs(yaw_error) > angular_deadband:
        angular_z = max(-max_angular, min(max_angular, gain * yaw_error))
    else:
        angular_z = 0.0
    return linear_x, linear_y, angular_z


def offset_to_reference(reference, current):
    """Map-frame position offset and wrapped yaw error toward a held pose.

    Both arguments are `((x, y), yaw)`.  Returns `(dx, dy, yaw_error)`.
    """
    (ref_x, ref_y), ref_yaw = reference
    (cur_x, cur_y), cur_yaw = current
    return ref_x - cur_x, ref_y - cur_y, wrap_angle(ref_yaw - cur_yaw)


def should_hold(command_is_fresh: bool, enabled: bool, pose_known: bool) -> bool:
    """Whether the watchdog should hold instead of publishing zero.

    Kept as a named predicate so the three conditions are visible at the call site and
    testable on their own: a fresh navigation command always wins, and without a pose
    there is nothing to hold.
    """
    return enabled and pose_known and not command_is_fresh
