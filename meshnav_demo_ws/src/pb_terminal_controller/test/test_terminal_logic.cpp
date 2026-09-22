// Copyright 2026. Licensed under Apache-2.0.
//
// Cases required by doc/PB_SLOPE_REPAIR_AND_DEPLOYMENT_PLAN.md section 6, all
// without a simulation because the logic is pure.

#include <gtest/gtest.h>

#include <cmath>

#include "pb_terminal_controller/terminal_logic.h"

using namespace pb_terminal_controller;  // NOLINT

namespace
{
TerminalParams params()
{
  return TerminalParams();
}

TerminalOutput advance(const TerminalParams & p, const TerminalInput & in,
                       TerminalOutput previous = TerminalOutput())
{
  return step(p, in, previous);
}

TerminalOutput in_state(TerminalState state)
{
  TerminalOutput output;
  output.state = state;
  return output;
}
}  // namespace

TEST(WrapAngle, HandlesThePlusMinusPiCrossing)
{
  EXPECT_NEAR(wrap_angle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(wrap_angle(M_PI - 0.01), M_PI - 0.01, 1e-12);
  // A small negative error written as 2*pi - 0.02 wraps back to -0.02.
  EXPECT_NEAR(wrap_angle(2.0 * M_PI - 0.02), -0.02, 1e-12);
  EXPECT_NEAR(wrap_angle(-2.0 * M_PI + 0.02), 0.02, 1e-12);
  // 3*pi is exactly halfway; std::remainder returns -pi, which is the same angle.
  EXPECT_NEAR(std::fabs(wrap_angle(3.0 * M_PI)), M_PI, 1e-12);
}

TEST(AngleFromDot, ClampsFloatingPointNoise)
{
  EXPECT_FALSE(std::isnan(angle_from_dot(1.0 + 1.0e-12)));
  EXPECT_NEAR(angle_from_dot(1.0 + 1.0e-12), 0.0, 1e-9);
  EXPECT_NEAR(angle_from_dot(-1.0 - 1.0e-12), M_PI, 1e-9);
  EXPECT_NEAR(angle_from_dot(0.0), M_PI / 2.0, 1e-12);
}

TEST(Arrival, RejectsTheHorizontalInsideVerticalOutsideCounterExample)
{
  // The historical 3-D check called this "arrived": sqrt(0.14^2 + 0.15^2) = 0.205 m
  // > 0.2 m, so it actually failed, and a horizontal-only check would wrongly
  // succeed.  A ground robot cannot fix 0.15 m of height, so it is a goal error.
  TerminalInput in;
  in.dx = 0.14;
  in.dy = 0.0;
  in.dz = 0.15;
  in.yaw_error = 0.0;
  EXPECT_FALSE(arrival_conditions_satisfied(params(), in));
  EXPECT_GT(std::hypot(in.dx, in.dy), 0.0);

  // Inside the level tolerance the same horizontal error is an arrival.
  in.dz = 0.02;
  EXPECT_TRUE(arrival_conditions_satisfied(params(), in));
}

TEST(Arrival, SamePlanarPositionOnAnotherLevelIsNotTheGoal)
{
  TerminalInput in;
  in.dx = 0.01;
  in.dy = 0.01;
  in.dz = 0.42 - 0.0;  // tunnel roof above the floor goal
  in.yaw_error = 0.0;
  EXPECT_FALSE(arrival_conditions_satisfied(params(), in));
}

TEST(GoalValid, WrongHeightIsFaultedInsteadOfChased)
{
  TerminalInput in;
  in.dx = 0.05;
  in.dy = 0.0;
  in.dz = 0.15;
  in.goal_valid = false;  // the goal handling layer rejected the height
  const auto out = advance(params(), in, in_state(TerminalState::TRACK));
  EXPECT_EQ(out.state, TerminalState::FAULT);
  EXPECT_EQ(out.linear_x, 0.0);
  EXPECT_EQ(out.angular_z, 0.0);
  EXPECT_FALSE(out.goal_reached);
  EXPECT_FALSE(out.message.empty());
}

TEST(StateMachine, TrackToSettleToAlignToHoldToFinished)
{
  TerminalParams p = params();
  p.settle_radius = 0.2;
  p.align_radius = 0.12;
  p.dwell_s = 0.5;

  TerminalInput in;
  in.dx = 1.0;
  in.tracking_linear_x = 0.5;
  in.dt = 0.1;
  auto out = advance(p, in, in_state(TerminalState::TRACK));
  EXPECT_EQ(out.state, TerminalState::TRACK);
  EXPECT_GT(out.linear_x, 0.0);

  in.dx = 0.15;
  out = step(p, in, out);
  EXPECT_EQ(out.state, TerminalState::POSITION_SETTLE);

  in.dx = 0.05;
  out = step(p, in, out);
  EXPECT_EQ(out.state, TerminalState::ALIGN_GOAL);
  EXPECT_EQ(out.linear_x, 0.0);
  EXPECT_EQ(out.linear_y, 0.0);

  in.yaw_error = 0.05;
  out = step(p, in, out);
  EXPECT_EQ(out.state, TerminalState::HOLD);
  EXPECT_FALSE(out.goal_reached);

  in.dt = 0.2;
  out = step(p, in, out);
  EXPECT_EQ(out.state, TerminalState::HOLD);
  EXPECT_FALSE(out.goal_reached) << "dwell time must be observed";

  in.dt = 0.4;
  out = step(p, in, out);
  EXPECT_EQ(out.state, TerminalState::FINISHED);
  EXPECT_TRUE(out.goal_reached);
  EXPECT_TRUE(out.finished);
}

TEST(StateMachine, SlidingAwayDuringAlignmentResumesPositionControl)
{
  TerminalParams p = params();
  p.align_radius = 0.12;
  p.hysteresis = 0.05;

  TerminalInput in;
  in.dx = 0.18;  // > align_radius + hysteresis
  const auto out = advance(p, in, in_state(TerminalState::ALIGN_GOAL));
  EXPECT_EQ(out.state, TerminalState::POSITION_SETTLE);
  EXPECT_NE(out.linear_x, 0.0) << "position control must resume";
}

TEST(StateMachine, CancelDuringAlignmentStopsAndReportsCanceling)
{
  TerminalInput in;
  in.dx = 0.05;
  in.yaw_error = 0.3;
  in.cancel_requested = true;
  const auto out = advance(params(), in, in_state(TerminalState::ALIGN_GOAL));
  EXPECT_EQ(out.state, TerminalState::CANCELING);
  EXPECT_EQ(out.linear_x, 0.0);
  EXPECT_EQ(out.linear_y, 0.0);
  EXPECT_EQ(out.angular_z, 0.0);
  EXPECT_FALSE(out.goal_reached);
}

TEST(StateMachine, UnhealthyInputBlocksAndThenRecovers)
{
  TerminalParams p = params();
  TerminalInput in;
  in.dx = 0.5;
  in.tracking_linear_x = 0.4;
  in.dt = 0.1;

  in.healthy = false;
  auto out = advance(p, in, in_state(TerminalState::TRACK));
  EXPECT_EQ(out.state, TerminalState::BLOCKED);
  EXPECT_EQ(out.linear_x, 0.0);

  // Recovery is delayed by recovery_dwell_s so that a block cannot flap straight
  // back into the condition that caused it.
  in.healthy = true;
  out = step(p, in, out);
  EXPECT_EQ(out.state, TerminalState::BLOCKED);
  while (out.state == TerminalState::BLOCKED && out.dwell <= p.recovery_dwell_s + in.dt) {
    out = step(p, in, out);
  }
  EXPECT_EQ(out.state, TerminalState::TRACK);
  EXPECT_GT(out.linear_x, 0.0);
}

TEST(StateMachine, DiscontinuousVectorFieldFallsBackToDrivingToTheGoal)
{
  // Near the goal the vector field can be discontinuous; the layer below then
  // commands nothing while the robot is still 0.15 m away.  Settling must not
  // depend on that field.
  TerminalInput in;
  in.dx = 0.0;
  in.dy = 0.15;
  in.robot_yaw = M_PI / 2.0;  // facing +Y, so the goal is straight ahead
  in.tracking_linear_x = 0.0;
  in.tracking_linear_y = 0.0;
  in.dt = 0.1;
  TerminalOutput out = in_state(TerminalState::POSITION_SETTLE);
  // The command ramps up under the acceleration limit, so advance several steps.
  for (int index = 0; index < 20; ++index) {
    out = step(params(), in, out);
    EXPECT_EQ(out.state, TerminalState::POSITION_SETTLE);
  }
  EXPECT_NEAR(out.linear_x, 0.15, 1e-6);
  EXPECT_NEAR(out.linear_y, 0.0, 1e-9);
}

TEST(Limits, VelocityAndAccelerationAreAlwaysRespected)
{
  TerminalParams p = params();
  p.max_linear_velocity = 0.5;
  p.max_angular_velocity = 0.4;
  p.max_linear_acceleration = 0.5;
  p.max_angular_acceleration = 1.0;

  TerminalInput in;
  in.dx = 5.0;
  in.tracking_linear_x = 5.0;
  in.tracking_angular_z = 3.0;
  in.dt = 0.1;

  TerminalOutput previous = in_state(TerminalState::TRACK);
  auto out = step(p, in, previous);
  EXPECT_LE(std::hypot(out.linear_x, out.linear_y), p.max_linear_velocity + 1e-12);
  EXPECT_LE(std::fabs(out.angular_z), p.max_angular_velocity + 1e-12);
  // Acceleration limit: at most max_accel * dt of change from the previous command.
  EXPECT_NEAR(out.linear_x, 0.05, 1e-9);
  EXPECT_NEAR(out.angular_z, 0.1, 1e-9);
}

TEST(Finished, AStaleSuccessIsDroppedWhenTheRobotDriftsAway)
{
  TerminalParams p = params();
  TerminalInput in;
  in.dx = 0.5;  // drifted well outside the settle band
  const auto out = advance(p, in, in_state(TerminalState::FINISHED));
  EXPECT_NE(out.state, TerminalState::FINISHED);
  EXPECT_FALSE(out.goal_reached);
}

// ---------------------------------------------------------------------------
// P3: a hold must be an active correction, because a zero command is not a stop on
// this force-based drive (T7 drifted 1.77 m over 95 s at zero command).
// ---------------------------------------------------------------------------

TEST(StationKeeping, HoldOpposesTheOffsetInsteadOfCommandingZero)
{
  TerminalParams p = params();
  TerminalInput in;
  in.dt = 0.1;
  in.dx = 0.05;            // 5 cm behind the held pose
  in.dy = -0.02;
  in.yaw_error = 0.05;

  TerminalOutput held;
  held.state = TerminalState::HOLD;
  const auto out = step(p, in, held);
  ASSERT_EQ(out.state, TerminalState::HOLD);
  // A non-zero, corrective command: no branch may stop the wheels.
  EXPECT_NE(out.linear_x, 0.0);
  EXPECT_NE(out.linear_y, 0.0);
  EXPECT_NE(out.angular_z, 0.0);
  // ...and it points back toward the held pose, not away from it.
  EXPECT_GT(out.linear_x * in.dx, 0.0);
  EXPECT_GT(out.linear_y * in.dy, 0.0);
  EXPECT_GT(out.angular_z * in.yaw_error, 0.0);
}

TEST(StationKeeping, HoldStaysInsideItsOwnVelocityBudget)
{
  TerminalParams p = params();
  p.hold_max_linear_velocity = 0.05;
  p.hold_max_angular_velocity = 0.1;
  TerminalInput in;
  in.dt = 0.1;
  in.dx = 5.0;             // a metre-scale offset must not become a lunge
  in.dy = 5.0;
  in.yaw_error = 3.0;

  TerminalOutput held;
  held.state = TerminalState::HOLD;
  const auto out = step(p, in, held);
  EXPECT_LE(std::hypot(out.linear_x, out.linear_y), p.hold_max_linear_velocity + 1e-9);
  EXPECT_LE(std::fabs(out.angular_z), p.hold_max_angular_velocity + 1e-9);
}

TEST(StationKeeping, FinishedKeepsHoldingAndStillReportsSuccess)
{
  TerminalParams p = params();
  TerminalInput in;
  in.dt = 0.1;
  in.dx = 0.04;
  in.yaw_error = 0.03;

  TerminalOutput previous;
  previous.state = TerminalState::FINISHED;
  const auto out = step(p, in, previous);
  EXPECT_EQ(out.state, TerminalState::FINISHED);
  EXPECT_TRUE(out.finished);
  EXPECT_TRUE(out.goal_reached);
  EXPECT_NE(std::hypot(out.linear_x, out.linear_y) + std::fabs(out.angular_z), 0.0);
}

TEST(StationKeeping, AlignDeadbandKeepsTheHoldFromBuzzing)
{
  TerminalParams p = params();
  TerminalInput in;
  in.dt = 0.1;
  in.yaw_error = p.align_deadband * 0.5;
  TerminalOutput held;
  held.state = TerminalState::HOLD;
  EXPECT_DOUBLE_EQ(step(p, in, held).angular_z, 0.0);
}

// ---------------------------------------------------------------------------
// P3: commanded-but-not-moving must be detected and reported, not pushed through
// for 25 s (the trap asked for ~47 N on average and the chassis never moved).
// ---------------------------------------------------------------------------

TEST(NoProgress, SustainedNoProgressBlocksAndStopsPushing)
{
  TerminalParams p = params();
  p.stuck_timeout_s = 1.0;
  TerminalInput in;
  in.dt = 0.1;
  in.dx = 3.0;                 // far from the goal, so TRACK stays active
  in.tracking_linear_x = 0.4;  // asking for motion
  in.moved_distance = 0.0;     // producing none

  TerminalOutput out;
  out.state = TerminalState::TRACK;
  out.linear_x = in.tracking_linear_x;   // the controller did ask for motion
  for (int i = 0; i < 9; ++i) {
    out = step(p, in, out);
    ASSERT_EQ(out.state, TerminalState::TRACK) << "tripped too early at step " << i;
  }
  out = step(p, in, out);
  EXPECT_EQ(out.state, TerminalState::BLOCKED);
  EXPECT_DOUBLE_EQ(out.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(out.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(out.angular_z, 0.0);
  EXPECT_FALSE(out.message.empty());
}

TEST(NoProgress, RealProgressResetsTheAccumulator)
{
  TerminalParams p = params();
  p.stuck_timeout_s = 0.25;
  TerminalInput in;
  in.dt = 0.1;
  in.dx = 3.0;
  in.tracking_linear_x = 0.4;

  TerminalOutput out;
  out.state = TerminalState::TRACK;
  out.linear_x = in.tracking_linear_x;
  in.moved_distance = 0.0;
  out = step(p, in, out);
  EXPECT_GT(out.stuck, 0.0);
  // A step that delivers the expected motion clears the accumulated suspicion.
  in.moved_distance = 0.4 * in.dt;
  out = step(p, in, out);
  EXPECT_DOUBLE_EQ(out.stuck, 0.0);
  EXPECT_EQ(out.state, TerminalState::TRACK);
}

TEST(NoProgress, HalfTheExpectedMotionStillCountsAsProgress)
{
  TerminalParams p = params();
  p.stuck_timeout_s = 5.0;
  TerminalInput in;
  in.dt = 0.1;
  in.dx = 3.0;
  in.tracking_linear_x = 0.4;
  in.moved_distance = 0.5 * 0.4 * in.dt;   // above the 0.25 fraction
  TerminalOutput out;
  out.state = TerminalState::TRACK;
  out.linear_x = in.tracking_linear_x;
  for (int i = 0; i < 20; ++i) {
    out = step(p, in, out);
  }
  EXPECT_DOUBLE_EQ(out.stuck, 0.0);
  EXPECT_EQ(out.state, TerminalState::TRACK);
}

TEST(NoProgress, AStationaryRobotWithNoCommandIsNeverBlocked)
{
  // Parking at zero command must not be reported as blocked: idle is not stuck.
  TerminalParams p = params();
  p.stuck_timeout_s = 0.2;
  TerminalInput in;
  in.dt = 0.1;
  in.dx = 0.1;
  in.moved_distance = 0.0;
  in.tracking_linear_x = 0.0;
  in.tracking_linear_y = 0.0;

  TerminalOutput out;
  out.state = TerminalState::HOLD;
  for (int i = 0; i < 30; ++i) {
    out = step(p, in, out);
  }
  EXPECT_TRUE(out.state == TerminalState::HOLD || out.state == TerminalState::FINISHED);
  EXPECT_DOUBLE_EQ(out.stuck, 0.0);
}
