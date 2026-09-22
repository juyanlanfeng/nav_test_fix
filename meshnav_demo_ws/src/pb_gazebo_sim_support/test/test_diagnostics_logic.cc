// Copyright 2026. Licensed under Apache-2.0.
//
// Unit tests for the ContactDiagnostics switches and message layout.  These need no
// simulation, which is the point: "default off" is the property that keeps the
// diagnostic from silently instrumenting every run, so it is tested directly rather
// than only observed in a live run.

#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <sdf/Element.hh>
#include <sdf/SDFImpl.hh>
#include <sdf/parser.hh>

#include "pb_gazebo_sim_support/diagnostics_logic.h"

namespace
{
/// \brief Parse a `<plugin>` element carrying the given child markup.
///
/// Uses the SDFPtr overload of readString: the ElementPtr overload takes the
/// pointer *by value*, so it cannot populate a caller's variable, and constructing
/// the element tree by hand crashes because AddValue needs an element description
/// that only a parser supplies.
std::shared_ptr<sdf::Element> PluginElement(const std::string &_children)
{
  const std::string xml =
      "<sdf version='1.9'><model name='m'><plugin filename='f' name='n'>" +
      _children + "</plugin></model></sdf>";
  sdf::SDFPtr sdf(new sdf::SDF());
  if (!sdf::init(sdf))
  {
    return nullptr;
  }
  if (!sdf::readString(xml, sdf))
  {
    return nullptr;
  }
  sdf::ElementPtr root = sdf->Root();
  if (root == nullptr)
  {
    return nullptr;
  }
  sdf::ElementPtr model = root->GetElement("model");
  if (model == nullptr)
  {
    return nullptr;
  }
  return model->GetElement("plugin");
}

/// \brief A `<plugin>` element with no children at all.
std::shared_ptr<sdf::Element> EmptyPluginElement()
{
  return PluginElement("");
}
}  // namespace

TEST(DiagnosticsLogic, AbsentSwitchStaysOff)
{
  const auto element = EmptyPluginElement();
  EXPECT_FALSE(pb_gazebo_sim_support::DiagnosticsEnabledFromElement(element, "enable", false));
}

TEST(DiagnosticsLogic, NullElementStaysOff)
{
  EXPECT_FALSE(pb_gazebo_sim_support::DiagnosticsEnabledFromElement(nullptr, "enable", false));
}

TEST(DiagnosticsLogic, ExplicitTrueEnables)
{
  for (const std::string value : {"true", "True", "TRUE", "1"})
  {
    const auto element = PluginElement("<enable>" + value + "</enable>");
    EXPECT_TRUE(pb_gazebo_sim_support::DiagnosticsEnabledFromElement(element, "enable", false))
        << "value: " << value;
  }
}

TEST(DiagnosticsLogic, ExplicitFalseDisablesEvenWhenDefaultIsOn)
{
  for (const std::string value : {"false", "False", "0"})
  {
    const auto element = PluginElement("<enable>" + value + "</enable>");
    EXPECT_FALSE(pb_gazebo_sim_support::DiagnosticsEnabledFromElement(element, "enable", true))
        << "value: " << value;
  }
}

TEST(DiagnosticsLogic, UnparseableValueFallsBackToTheDefault)
{
  const auto element = PluginElement("<enable>yes-please</enable>");
  EXPECT_FALSE(pb_gazebo_sim_support::DiagnosticsEnabledFromElement(element, "enable", false));
  EXPECT_TRUE(pb_gazebo_sim_support::DiagnosticsEnabledFromElement(element, "enable", true));
}

TEST(DiagnosticsLogic, SubSwitchesDefaultOnOnlyOnceTheMasterIsOn)
{
  const auto element = EmptyPluginElement();
  // The sub-switches default to on; the master switch is what keeps the whole
  // plugin inert, so this only documents the two-level default.
  EXPECT_TRUE(pb_gazebo_sim_support::DiagnosticsEnabledFromElement(element, "enable_actuator", true));
  EXPECT_TRUE(pb_gazebo_sim_support::DiagnosticsEnabledFromElement(element, "enable_contact", true));
}

TEST(DiagnosticsLogic, JointListParsesSpacesCommasAndTrailingSeparators)
{
  const auto names = pb_gazebo_sim_support::ParseJointList(
      " front_left_wheel_joint, front_right_wheel_joint  rear_left_wheel_joint ,");
  ASSERT_EQ(names.size(), 3u);
  EXPECT_EQ(names[0], "front_left_wheel_joint");
  EXPECT_EQ(names[1], "front_right_wheel_joint");
  EXPECT_EQ(names[2], "rear_left_wheel_joint");
  EXPECT_TRUE(pb_gazebo_sim_support::ParseJointList("").empty());
  EXPECT_TRUE(pb_gazebo_sim_support::ParseJointList("  ,,  ").empty());
}

TEST(DiagnosticsLogic, JointStateLayoutIsVelocityThenPositionThenForce)
{
  const auto layout = pb_gazebo_sim_support::JointStateLayout({1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0});
  ASSERT_EQ(layout.size(), 6u);
  EXPECT_DOUBLE_EQ(layout[0], 1.0);
  EXPECT_DOUBLE_EQ(layout[1], 2.0);
  EXPECT_DOUBLE_EQ(layout[2], 3.0);
  EXPECT_DOUBLE_EQ(layout[3], 4.0);
  EXPECT_DOUBLE_EQ(layout[4], 5.0);
  EXPECT_DOUBLE_EQ(layout[5], 6.0);
}

TEST(DiagnosticsLogic, JointStateLayoutToleratesMissingJoints)
{
  const auto layout = pb_gazebo_sim_support::JointStateLayout({}, {7.0}, {});
  ASSERT_EQ(layout.size(), 1u);
  EXPECT_DOUBLE_EQ(layout[0], 7.0);
}

TEST(DiagnosticsLogic, MotionRequestIgnoresFloatingPointNoise)
{
  EXPECT_FALSE(pb_gazebo_sim_support::RequestsMotion(0.0));
  EXPECT_FALSE(pb_gazebo_sim_support::RequestsMotion(1.0e-9));
  EXPECT_TRUE(pb_gazebo_sim_support::RequestsMotion(0.5));
  EXPECT_TRUE(pb_gazebo_sim_support::RequestsMotion(-0.5));
}
