// Copyright 2026. Licensed under Apache-2.0.
#include "pb_terminal_controller/terminal_logic.h"

#include <algorithm>
#include <cmath>

namespace pb_terminal_controller
{

const char * to_string(TerminalState state)
{
  switch (state) {
    case TerminalState::TRACK: return "TRACK";
    case TerminalState::POSITION_SETTLE: return "POSITION_SETTLE";
    case TerminalState::ALIGN_GOAL: return "ALIGN_GOAL";
    case TerminalState::HOLD: return "HOLD";
    case TerminalState::FINISHED: return "FINISHED";
    case TerminalState::CANCELING: return "CANCELING";
    case TerminalState::BLOCKED: return "BLOCKED";
    case TerminalState::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

double wrap_angle(double angle)
{
  // std::remainder returns a value in [-pi, pi] and handles the +/-pi crossing.
  return std::remainder(angle, 2.0 * M_PI);
}

double clamp_unit(double value)
{
  return std::max(-1.0, std::min(1.0, value));
}

double angle_from_dot(double dot)
{
  return std::acos(clamp_unit(dot));
}

double planar_error(const TerminalInput & in)
{
  return std::hypot(in.dx, in.dy);
}

bool arrival_conditions_satisfied(const TerminalParams & params, const TerminalInput & in)
{
  // One criterion, used by the state machine and by the caller's arrival check:
  // position inside the settle radius, the goal on the robot's level, and the
  // heading aligned.  A ground robot cannot correct height with a Z velocity, so
  // the level term is a *goal* condition, never something the controller chases.
  return in.goal_valid && in.healthy &&
         planar_error(in) <= params.settle_radius &&
         std::fabs(in.dz) <= params.level_tolerance &&
         std::fabs(in.yaw_error) <= params.yaw_tolerance;
}

namespace
{

double limit_acceleration(double target, double previous, double max_acceleration, double dt)
{
  if (dt <= 0.0) {
    return target;
  }
  const double maximum_change = max_acceleration * dt;
  return previous + std::clamp(target - previous, -maximum_change, maximum_change);
}

void stop(TerminalOutput & out)
{
  out.linear_x = 0.0;
  out.linear_y = 0.0;
  out.angular_z = 0.0;
}

/// Body-frame velocity toward the goal, used when the layer below produces nothing
/// usable (for example a discontinuous vector field right at the goal).
void drive_toward_goal(const TerminalParams & params, const TerminalInput & in, TerminalOutput & out)
{
  const double cos_yaw = std::cos(in.robot_yaw);
  const double sin_yaw = std::sin(in.robot_yaw);
  const double forward = cos_yaw * in.dx + sin_yaw * in.dy;
  const double lateral = -sin_yaw * in.dx + cos_yaw * in.dy;
  out.linear_x = std::clamp(forward, -params.settle_max_linear_velocity,
                            params.settle_max_linear_velocity);
  out.linear_y = std::clamp(lateral, -params.settle_max_linear_velocity,
                            params.settle_max_linear_velocity);
  out.angular_z = std::clamp(params.align_gain * in.yaw_error,
                             -params.settle_max_angular_velocity,
                             params.settle_max_angular_velocity);
}

}  // namespace

void station_keeping_command(const TerminalParams & params, const TerminalInput & in,
                             TerminalOutput & out)
{
  // Oppose the measured offset from the held pose with a bounded velocity.  The
  // offset is already expressed in the map frame, so this needs no map knowledge
  // beyond what the arrival criterion already uses.
  const double cos_yaw = std::cos(in.robot_yaw);
  const double sin_yaw = std::sin(in.robot_yaw);
  const double forward = cos_yaw * in.dx + sin_yaw * in.dy;
  const double lateral = -sin_yaw * in.dx + cos_yaw * in.dy;
  out.linear_x = std::clamp(params.hold_gain * forward,
                            -params.hold_max_linear_velocity,
                            params.hold_max_linear_velocity);
  out.linear_y = std::clamp(params.hold_gain * lateral,
                            -params.hold_max_linear_velocity,
                            params.hold_max_linear_velocity);
  out.angular_z = std::fabs(in.yaw_error) > params.align_deadband
                    ? std::clamp(params.hold_gain * in.yaw_error,
                                 -params.hold_max_angular_velocity,
                                 params.hold_max_angular_velocity)
                    : 0.0;
}

namespace
{

void apply_limits(const TerminalParams & params, const TerminalInput & in,
                  const TerminalOutput & previous, TerminalOutput & out)
{
  // Velocity limits first, then the acceleration limit against the previous command.
  const double linear_norm = std::hypot(out.linear_x, out.linear_y);
  if (linear_norm > params.max_linear_velocity) {
    const double scale = params.max_linear_velocity / linear_norm;
    out.linear_x *= scale;
    out.linear_y *= scale;
  }
  out.angular_z = std::clamp(out.angular_z, -params.max_angular_velocity,
                             params.max_angular_velocity);

  const double previous_linear_norm = std::hypot(previous.linear_x, previous.linear_y);
  const double limited_linear_norm = limit_acceleration(
    std::hypot(out.linear_x, out.linear_y), previous_linear_norm,
    params.max_linear_acceleration, in.dt);
  if (std::hypot(out.linear_x, out.linear_y) > 1.0e-9) {
    const double scale = limited_linear_norm / std::hypot(out.linear_x, out.linear_y);
    out.linear_x *= scale;
    out.linear_y *= scale;
  }
  out.angular_z = limit_acceleration(out.angular_z, previous.angular_z,
                                     params.max_angular_acceleration, in.dt);
}

}  // namespace

TerminalOutput step(const TerminalParams & params, const TerminalInput & in,
                    const TerminalOutput & previous)
{
  TerminalOutput out;
  out.state = previous.state;
  out.dwell = previous.dwell;
  out.stuck = previous.stuck;

  if (!in.goal_valid) {
    out.state = TerminalState::FAULT;
    out.dwell = 0.0;
    stop(out);
    out.message = "goal rejected: not on a reachable support surface at the robot's level";
    return out;
  }
  if (in.cancel_requested) {
    out.state = TerminalState::CANCELING;
    out.dwell = 0.0;
    stop(out);
    out.message = "cancel requested";
    return out;
  }
  if (!in.healthy) {
    out.state = TerminalState::BLOCKED;
    out.dwell = 0.0;
    stop(out);
    out.message = "input unhealthy: holding still";
    return out;
  }

  // ---- commanded but not moving: judge the *previous* command against the motion it
  // actually produced.  A step where the robot was not asked to move never counts, so
  // parking at zero command cannot be reported as blocked.
  // Only TRACK is judged.  The other motion states issue deliberate low-speed
  // corrections (settle, align, and the station-keeping hold), where producing no net
  // translation is the expected outcome — judging them would turn a legitimate hold
  // into a false "blocked".
  const double asked = std::hypot(previous.linear_x, previous.linear_y);
  const double expected = asked * std::max(0.0, in.dt);
  const bool judging = previous.state == TerminalState::TRACK &&
                       asked >= params.stuck_command_threshold &&
                       expected >= params.stuck_min_expected_m;
  if (judging && in.moved_distance < params.stuck_progress_fraction * expected) {
    out.stuck = previous.stuck + std::max(0.0, in.dt);
  } else if (judging) {
    out.stuck = 0.0;
  }
  // Epsilon because the accumulator adds dt: ten 0.1 s steps sum to 0.9999999999999999,
  // and a timeout that can be missed by one ulp is a latent bug.
  if (out.stuck + 1.0e-9 >= params.stuck_timeout_s) {
    out.state = TerminalState::BLOCKED;
    out.dwell = 0.0;
    stop(out);
    out.message = "commanded motion produced no progress; stopped pushing and holding";
    return out;
  }

  const double distance = planar_error(in);
  const bool level_ok = std::fabs(in.dz) <= params.level_tolerance;
  const bool conditions = arrival_conditions_satisfied(params, in);
  const double align_exit = params.align_radius + params.hysteresis;
  const double settle_exit = params.settle_radius + params.hysteresis;
  const double yaw_exit = params.yaw_tolerance + params.yaw_hysteresis;

  switch (previous.state) {
    case TerminalState::TRACK:
      if (level_ok && distance <= params.settle_radius) {
        out.state = TerminalState::POSITION_SETTLE;
        out.dwell = 0.0;
      }
      break;
    case TerminalState::POSITION_SETTLE:
      if (!level_ok || distance > settle_exit) {
        out.state = TerminalState::TRACK;
        out.dwell = 0.0;
      } else if (distance <= params.align_radius) {
        out.state = TerminalState::ALIGN_GOAL;
        out.dwell = 0.0;
      }
      break;
    case TerminalState::ALIGN_GOAL:
      if (!level_ok || distance > align_exit) {
        // Sliding away during alignment resumes position control instead of
        // rotating in place forever.
        out.state = TerminalState::POSITION_SETTLE;
        out.dwell = 0.0;
      } else if (conditions) {
        // The trigger for the hold uses the *same* criterion as the arrival check,
        // so the two can never disagree.
        out.state = TerminalState::HOLD;
        out.dwell = 0.0;
      }
      break;
    case TerminalState::HOLD:
      if (!level_ok || distance > align_exit) {
        out.state = TerminalState::POSITION_SETTLE;
        out.dwell = 0.0;
      } else if (std::fabs(in.yaw_error) > yaw_exit) {
        out.state = TerminalState::ALIGN_GOAL;
        out.dwell = 0.0;
      } else {
        out.dwell = previous.dwell + std::max(0.0, in.dt);
        if (out.dwell >= params.dwell_s) {
          out.state = TerminalState::FINISHED;
        }
      }
      break;
    case TerminalState::BLOCKED:
      // Resume tracking once the inputs are healthy, but only after a short dwell:
      // re-entering immediately would flap straight back into the same no-progress
      // condition without giving the caller a chance to re-plan.
      if (in.healthy) {
        out.dwell = previous.dwell + std::max(0.0, in.dt);
        if (out.dwell >= params.recovery_dwell_s) {
          out.state = TerminalState::TRACK;
          out.dwell = 0.0;
          out.stuck = 0.0;
          out.message = "resuming tracking after the no-progress stop";
        }
      } else {
        out.dwell = 0.0;
      }
      break;
    case TerminalState::FINISHED:
      // A finished goal is only kept while the conditions still hold; if the robot
      // drifts or slides away, position control resumes instead of reporting a
      // stale success.
      if (!level_ok || distance > settle_exit) {
        out.state = TerminalState::TRACK;
        out.dwell = 0.0;
      } else {
        // Success is reported, but the vehicle is still held: returning here with a
        // zero command is what let the goal pose drift (T7).
        out.finished = true;
        out.goal_reached = true;
      }
      break;
    default:
      break;
  }

  switch (out.state) {
    case TerminalState::TRACK:
      // The layer below owns the motion; the terminal layer only limits it.
      out.linear_x = in.tracking_linear_x;
      out.linear_y = in.tracking_linear_y;
      out.angular_z = in.tracking_angular_z;
      break;
    case TerminalState::POSITION_SETTLE:
      if (std::hypot(in.tracking_linear_x, in.tracking_linear_y) > 1.0e-3) {
        out.linear_x = in.tracking_linear_x;
        out.linear_y = in.tracking_linear_y;
        out.angular_z = in.tracking_angular_z;
      } else {
        drive_toward_goal(params, in, out);
      }
      break;
    case TerminalState::ALIGN_GOAL:
      stop(out);
      if (std::fabs(in.yaw_error) > params.align_deadband) {
        out.angular_z = params.align_gain * in.yaw_error;
      }
      break;
    case TerminalState::HOLD:
    case TerminalState::FINISHED:
      // A hold is an active, bounded correction.  On this force-based drive a zero
      // command applies no brake, so stopping the command is not stopping the robot.
      stop(out);
      station_keeping_command(params, in, out);
      break;
    default:
      stop(out);
      break;
  }

  apply_limits(params, in, previous, out);
  if (out.state == TerminalState::FINISHED) {
    out.finished = true;
    out.goal_reached = true;
  }
  return out;
}

}  // namespace pb_terminal_controller
