// Copyright 2026. Licensed under Apache-2.0.
//
// Pure terminal-control logic for a ground robot: no ROS, no Gazebo, no mesh
// library.  Keeping it pure is what makes the cases the repair plan demands
// testable without a simulation: the 0.14/0.15 m counter-example, the same-XY
// different-level goal, a wrong goal height, headings that cross +/-pi, a goal
// whose position is satisfied while the robot slides away, and cancel arriving
// during alignment.
//
// The state machine is deliberately generic: it never mentions a ramp, a tunnel or
// any other scene feature, so it can be used from a MeshNav controller plugin, a
// SCAN tracker or a real-robot task layer.
//
//   TRACK -> POSITION_SETTLE -> ALIGN_GOAL -> HOLD -> FINISHED
//                                  ^   |         (dwell time)
//                                  |   v  (hysteresis: leaving the align band
//                                  +---+)  returns to position control)
//   CANCELING and BLOCKED and FAULT are reachable from any state.

#ifndef PB_TERMINAL_CONTROLLER__TERMINAL_LOGIC_H
#define PB_TERMINAL_CONTROLLER__TERMINAL_LOGIC_H

#include <string>

namespace pb_terminal_controller
{

enum class TerminalState
{
  TRACK,
  POSITION_SETTLE,
  ALIGN_GOAL,
  HOLD,
  FINISHED,
  CANCELING,
  BLOCKED,
  FAULT,
};

const char * to_string(TerminalState state);

/// \brief Limits and thresholds.  Declared before a run, never relaxed afterwards.
struct TerminalParams
{
  /// Planar distance at which position settling starts [m].
  double settle_radius = 0.2;
  /// Planar distance at which in-place alignment starts [m].
  double align_radius = 0.12;
  /// Extra distance that must be exceeded to leave ALIGN_GOAL again [m].
  double hysteresis = 0.05;
  /// Allowed |robot z - goal z on the current level| [m].  A ground robot cannot
  /// correct height with a Z velocity, so exceeding this is a goal error, not a
  /// control problem.
  double level_tolerance = 0.1;
  /// Yaw tolerance that counts as aligned [rad].
  double yaw_tolerance = 0.1;
  /// Extra yaw error that must be exceeded to leave HOLD again [rad].
  double yaw_hysteresis = 0.05;
  /// Time the robot must hold position and heading before FINISHED [s].
  double dwell_s = 1.0;
  /// P gain for the in-place alignment [1/s].
  double align_gain = 1.5;
  /// Yaw error below which no rotation is commanded [rad].
  double align_deadband = 0.02;
  /// Speed and acceleration limits (the robot's own capability, not a scene
  /// parameter).
  double max_linear_velocity = 1.0;
  double max_angular_velocity = 0.5;
  double max_linear_acceleration = 0.5;
  double max_angular_acceleration = 1.0;
  /// Speeds allowed while settling the position [m/s].
  /// Station keeping.  Zero command is not a hold on this vehicle: the force-based
  /// drive applies no brake at zero, so HOLD/FINISHED must actively correct the
  /// measured offset through a bounded closed-loop command, exactly like a real
  /// position-hold loop.
  double hold_gain = 1.0;
  double hold_max_linear_velocity = 0.1;
  double hold_max_angular_velocity = 0.2;

  /// No-progress (stuck) detection.  While the layer below asks for motion, compare
  /// the distance the body actually travelled against what that command should have
  /// produced; sustained failure to progress means the robot is pushing against
  /// something it cannot overcome, and continuing to push only burns the actuator.
  double stuck_command_threshold = 0.05;   ///< command above this counts as "asking" [m/s]
  double stuck_progress_fraction = 0.25;   ///< fraction of the expected motion required
  double stuck_min_expected_m = 0.01;      ///< ignore steps too small to judge [m]
  double stuck_timeout_s = 8.0;            ///< sustained no-progress time that trips [s]
  double recovery_dwell_s = 1.0;           ///< time held in BLOCKED before resuming [s]

  double settle_max_linear_velocity = 0.3;
  double settle_max_angular_velocity = 0.3;
};

/// \brief One control step input: everything measured, nothing commanded.
struct TerminalInput
{
  double dx = 0.0;             ///< goal_x - robot_x in the map frame [m]
  double dy = 0.0;             ///< goal_y - robot_y in the map frame [m]
  double dz = 0.0;             ///< goal_z - robot_z on the current level [m]
  double yaw_error = 0.0;      ///< wrapped goal yaw - robot yaw [rad]
  double robot_yaw = 0.0;      ///< current robot yaw in the map frame [rad]
  double dt = 0.0;             ///< time since the previous step [s]
  /// Distance the body actually travelled since the previous step [m].  Measured, not
  /// commanded: this is what makes the stuck test a statement about the world rather
  /// than about the controller's intentions.
  double moved_distance = 0.0;
  bool healthy = true;         ///< input health (localization, truth chain...)
  bool cancel_requested = false;
  bool goal_valid = true;      ///< false when the goal failed the level check
  /// Tracking command produced by the layer below (vector field / trajectory
  /// tracker).  The terminal layer only overrides it near the goal.
  double tracking_linear_x = 0.0;
  double tracking_linear_y = 0.0;
  double tracking_angular_z = 0.0;
};

/// \brief One control step output.
struct TerminalOutput
{
  TerminalState state = TerminalState::TRACK;
  double linear_x = 0.0;
  double linear_y = 0.0;
  double angular_z = 0.0;
  double dwell = 0.0;          ///< accumulated hold time [s]
  double stuck = 0.0;          ///< accumulated commanded-but-no-progress time [s]
  bool finished = false;
  bool goal_reached = false;
  std::string message;
};

/// \brief Wrap an angle into (-pi, pi].
double wrap_angle(double angle);

/// \brief Clamp an acos argument: floating point noise can push it outside [-1, 1].
double clamp_unit(double value);

/// \brief Angle between two directions given their dot product, safe near +/-1.
double angle_from_dot(double dot);

/// \brief Planar distance error, the quantity the arrival criterion uses.
double planar_error(const TerminalInput & in);

/// \brief Whether position, level and heading are all satisfied (the single
/// arrival criterion used by both the state machine and the caller).
bool arrival_conditions_satisfied(const TerminalParams & params, const TerminalInput & in);

/// \brief Bounded closed-loop hold command for the current pose offset.
///
/// Used by HOLD and FINISHED instead of a zero command.  Pure and side-effect free so
/// the hold behaviour can be tested without a simulation.
void station_keeping_command(const TerminalParams & params, const TerminalInput & in,
                             TerminalOutput & out);

/// \brief Advance the state machine by one step.
///
/// `previous` carries the state and the accumulated dwell time; the returned output
/// is the next state and the command to publish.  Velocity and acceleration limits
/// are applied here, so no reachable state can exceed them.
TerminalOutput step(const TerminalParams & params, const TerminalInput & in,
                    const TerminalOutput & previous);

}  // namespace pb_terminal_controller

#endif  // PB_TERMINAL_CONTROLLER__TERMINAL_LOGIC_H
