#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "xlerobot_manipulation/policy_replay.hpp"

namespace xm = xlerobot_manipulation;

namespace
{

xm::StreamSafetyConfig config()
{
  return {
    {
      {"right_arm_shoulder_pan", -2.05, 2.05},
      {"right_arm_shoulder_lift", -1.40, 1.85},
      {"right_arm_elbow_flex", -1.65, 1.70},
      {"right_arm_wrist_flex", -1.75, 1.75},
      {"right_arm_wrist_roll", -3.09, 3.09},
      {"right_arm_gripper", 0.0, 1.65,
        xm::JointCommandSemantics::kPositionSetpoint},
    },
    0.02, 0.10, 0.25, 0.05, 2.0};
}

std::string fixture(const std::string & name)
{
  return std::string(TEST_DATA_DIR) + "/" + name;
}

}  // namespace

TEST(PolicyReplayTest, LoadsVersionedProvenanceAndAcceptsSafeFixture)
{
  const auto replay = xm::load_policy_replay(fixture("policy_replay_safe_synthetic.yaml"));
  EXPECT_EQ(replay.format, "xlerobot_policy_replay/v1");
  EXPECT_EQ(replay.provenance_kind, "synthetic_contract_fixture");
  EXPECT_EQ(replay.chunks.size(), 2U);
  const auto result = xm::validate_policy_replay(replay, config());
  EXPECT_TRUE(result.accepted) << result.message;
  EXPECT_EQ(result.accepted_chunks, 2U);
}

TEST(PolicyReplayTest, RejectsUnsafeFixtureWithoutPartialAcceptance)
{
  const auto replay = xm::load_policy_replay(fixture("policy_replay_unsafe_synthetic.yaml"));
  const auto result = xm::validate_policy_replay(replay, config());
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.accepted_chunks, 0U);
  EXPECT_EQ(result.fault, xm::StreamFault::kPositionLimit);
}

TEST(PolicyReplayTest, AcceptsFrozenVerifiedQuarterSecondGripperSemantics)
{
  const auto replay = xm::load_policy_replay(
    fixture("policy_replay_verified_gripper_ramp.yaml"));
  EXPECT_EQ(replay.provenance_kind, "frozen_verified_governor_fixture");
  const auto result = xm::validate_policy_replay(replay, config());
  EXPECT_TRUE(result.accepted) << result.message;
  EXPECT_EQ(result.accepted_chunks, 2U);
}

TEST(PolicyReplayTest, RejectsUnknownFormat)
{
  EXPECT_THROW(
    xm::load_policy_replay(fixture("policy_replay_bad_format.yaml")),
    std::runtime_error);
}
