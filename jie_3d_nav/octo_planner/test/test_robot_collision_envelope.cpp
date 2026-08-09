#include <gtest/gtest.h>

#include "octo_planner/robot_collision_envelope.hpp"

TEST(RobotCollisionEnvelope, PreservesLegacyHemisphereWhenNewParametersAreUnset)
{
  const auto envelope =
    octo_planner::resolveRobotCollisionEnvelope(0.25, -1.0, -1.0);

  EXPECT_TRUE(envelope.use_legacy_hemisphere);
  EXPECT_TRUE(octo_planner::collisionEnvelopeContainsOffset(envelope, 3, 0, 4, 0.05));
  EXPECT_FALSE(octo_planner::collisionEnvelopeContainsOffset(envelope, 4, 0, 4, 0.05));
}

TEST(RobotCollisionEnvelope, UsesAnisotropicCylinderForRmucRobot)
{
  const auto envelope =
    octo_planner::resolveRobotCollisionEnvelope(0.25, 0.28, 0.225);

  EXPECT_FALSE(envelope.use_legacy_hemisphere);
  // Direct support is one cell below the candidate. dz=3 is the fourth
  // 0.05 m cell above ground (0.20 m); dz=4 is 0.25 m and outside H=0.225.
  EXPECT_TRUE(octo_planner::collisionEnvelopeContainsOffset(envelope, 5, 0, 3, 0.05, 1));
  EXPECT_FALSE(octo_planner::collisionEnvelopeContainsOffset(envelope, 6, 0, 0, 0.05));
  EXPECT_FALSE(octo_planner::collisionEnvelopeContainsOffset(envelope, 0, 0, 4, 0.05, 1));
}

TEST(RobotCollisionEnvelope, QuantizedClearanceBoundaryMatchesPhysicalHeight)
{
  const auto envelope =
    octo_planner::resolveRobotCollisionEnvelope(0.25, 0.28, 0.225);

  // At 0.1 m, the first cell above a direct candidate is 0.20 m above
  // support and must collide; the following one is 0.30 m and must not.
  EXPECT_TRUE(octo_planner::collisionEnvelopeContainsOffset(envelope, 0, 0, 1, 0.10, 1));
  EXPECT_FALSE(octo_planner::collisionEnvelopeContainsOffset(envelope, 0, 0, 2, 0.10, 1));

  // If support is two cells below the candidate, the body begins at dz=-1.
  EXPECT_TRUE(octo_planner::collisionEnvelopeContainsOffset(envelope, 0, 0, -1, 0.05, 2));
  EXPECT_TRUE(octo_planner::collisionEnvelopeContainsOffset(envelope, 0, 0, 2, 0.05, 2));
  EXPECT_FALSE(octo_planner::collisionEnvelopeContainsOffset(envelope, 0, 0, 3, 0.05, 2));
}

TEST(RobotCollisionEnvelope, ExactTwentyCentimeterBoundaryIsIncluded)
{
  const auto envelope =
    octo_planner::resolveRobotCollisionEnvelope(0.25, 0.28, 0.20);

  // Direct support is one voxel below the candidate: relative dz=3 is the
  // cell whose centre is exactly 0.20 m above support and therefore belongs
  // to the physical body.  dz=4 is 0.25 m and lies outside it.
  EXPECT_TRUE(octo_planner::collisionEnvelopeContainsOffset(envelope, 0, 0, 3, 0.05, 1));
  EXPECT_FALSE(octo_planner::collisionEnvelopeContainsOffset(envelope, 0, 0, 4, 0.05, 1));
}

TEST(RobotCollisionEnvelope, HeightBelowResolutionHasNoAdditionalBodyCell)
{
  const auto envelope =
    octo_planner::resolveRobotCollisionEnvelope(0.25, 0.28, 0.04);

  // The planner rejects an occupied/preblocked candidate before evaluating
  // the envelope.  This helper must not invent a full extra vertical voxel
  // when the configured physical height is below the map resolution.
  EXPECT_FALSE(octo_planner::collisionEnvelopeContainsOffset(envelope, 0, 0, 0, 0.05, 1));
}

TEST(RobotCollisionEnvelope, RmucTunnelPassesWhileTwentyCentimeterGapIsRejected)
{
  const auto envelope =
    octo_planner::resolveRobotCollisionEnvelope(0.25, 0.28, 0.225);

  // With the canonical centre-quantized PCD at 0.05 m, the approximately
  // 0.246 m tunnel underside is candidate-relative dz=4 (0.25 m above
  // support), so it stays outside the body. A 0.20 m underside is dz=3 and
  // intersects the configured 0.225 m physical envelope.
  EXPECT_FALSE(octo_planner::collisionEnvelopeContainsOffset(envelope, 0, 0, 4, 0.05, 1));
  EXPECT_TRUE(octo_planner::collisionEnvelopeContainsOffset(envelope, 0, 0, 3, 0.05, 1));
}

TEST(RobotCollisionEnvelope, SupportDisabledStillChecksCandidateLevelFootprint)
{
  const auto envelope =
    octo_planner::resolveRobotCollisionEnvelope(0.25, 0.28, 0.225);

  EXPECT_TRUE(octo_planner::collisionEnvelopeContainsOffset(envelope, 5, 0, 0, 0.05, 0));
  EXPECT_FALSE(octo_planner::collisionEnvelopeContainsOffset(envelope, 6, 0, 0, 0.05, 0));
}

TEST(RobotCollisionEnvelope, NegativeGridPreblockedGapIsNotSkipped)
{
  const auto no_occupied = [](int) {return false;};
  const auto preblocked_minus_one = [](int z) {return z == -1;};
  EXPECT_TRUE(
    octo_planner::verticalColumnHasPreblockedGap(
      0, -2, no_occupied, preblocked_minus_one));

  // The first occupied cell terminates the scan; a deeper preblocked cell is
  // below valid direct support and must not reject the candidate.
  const auto occupied_minus_one = [](int z) {return z == -1;};
  const auto preblocked_minus_two = [](int z) {return z == -2;};
  EXPECT_FALSE(
    octo_planner::verticalColumnHasPreblockedGap(
      0, -2, occupied_minus_one, preblocked_minus_two));
}

TEST(RobotCollisionEnvelope, FallsBackPerAxisToLegacyRadius)
{
  const auto envelope =
    octo_planner::resolveRobotCollisionEnvelope(0.20, 0.28, -1.0);

  EXPECT_FALSE(envelope.use_legacy_hemisphere);
  EXPECT_DOUBLE_EQ(envelope.radius_xy, 0.28);
  EXPECT_DOUBLE_EQ(envelope.height_from_support, 0.20);
}

TEST(RobotCollisionEnvelope, SupportCandidateOffsetsMatchSupportSearch)
{
  const auto strict = octo_planner::groundSupportCandidateOffsets(true, 7, 9);
  ASSERT_EQ(strict.size(), 1U);
  EXPECT_EQ(strict.front().x, 0);
  EXPECT_EQ(strict.front().y, 0);
  EXPECT_EQ(strict.front().z, 1);

  const auto relaxed = octo_planner::groundSupportCandidateOffsets(false, 1, 2);
  EXPECT_EQ(relaxed.size(), 18U);
  bool has_near = false;
  bool has_deep = false;
  for (const auto & offset : relaxed) {
    has_near = has_near || (offset.x == -1 && offset.y == 1 && offset.z == 1);
    has_deep = has_deep || (offset.x == 1 && offset.y == -1 && offset.z == 2);
  }
  EXPECT_TRUE(has_near);
  EXPECT_TRUE(has_deep);
  EXPECT_TRUE(octo_planner::groundSupportCandidateOffsets(false, -1, 2).empty());
}
