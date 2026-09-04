#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "xlerobot_manipulation/streaming_validator.hpp"

namespace xm = xlerobot_manipulation;

namespace
{

xm::StreamSafetyConfig config()
{
  return {
    {
      {"shoulder", -1.0, 1.0},
      {"gripper", 0.0, 1.5},
    },
    0.02,
    0.10,
    0.25,
    0.05,
    0.50,
  };
}

xm::PolicyChunk valid_chunk(uint64_t sequence = 0)
{
  return {
    "session-1",
    "act-policy",
    sequence,
    0.02,
    0.05,
    false,
    {"shoulder", "gripper"},
    2,
    {0.02, 0.51, 0.04, 0.52},
  };
}

class StreamingValidatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_TRUE(validator_.begin("session-1", "act-policy", {0.0, 0.5}, {0.0, 0.0}));
  }

  xm::StreamingValidator validator_{config()};
};

}  // namespace

TEST(StreamingValidatorConfigTest, RejectsInvalidEnvelopeAndStartState)
{
  auto invalid = config();
  invalid.joints[0].lower_position = 1.0;
  xm::StreamingValidator validator(invalid);
  EXPECT_EQ(
    validator.begin("session", "source", {0.0, 0.5}, {0.0, 0.0}).fault,
    xm::StreamFault::kInvalidConfig);

  xm::StreamingValidator valid(config());
  EXPECT_EQ(
    valid.begin("session", "source", {2.0, 0.5}, {0.0, 0.0}).fault,
    xm::StreamFault::kPositionLimit);
  EXPECT_EQ(
    valid.begin("session", "source", {0.0, 0.5}, {0.0}).fault,
    xm::StreamFault::kDimensions);
  EXPECT_FALSE(valid.active());
}

TEST_F(StreamingValidatorTest, AcceptsOrderedChunksAndFinalMarker)
{
  EXPECT_TRUE(validator_.validate_and_accept(valid_chunk(), 0.0));
  EXPECT_EQ(validator_.next_sequence(), 1U);
  EXPECT_EQ(validator_.last_positions(), (std::vector<double>{0.04, 0.52}));

  auto final = valid_chunk(1);
  final.sample_count = 1;
  final.positions = {0.06, 0.53};
  final.final_chunk = true;
  EXPECT_TRUE(validator_.validate_and_accept(final, 0.05));
  EXPECT_TRUE(validator_.final_received());
  EXPECT_EQ(
    validator_.validate_and_accept(valid_chunk(2), 0.0).fault,
    xm::StreamFault::kAlreadyFinal);
}

TEST_F(StreamingValidatorTest, RejectsIdentityOrderingAndTimeFailures)
{
  auto chunk = valid_chunk();
  chunk.session_id = "wrong";
  EXPECT_EQ(
    validator_.validate_and_accept(chunk, 0.0).fault,
    xm::StreamFault::kWrongSession);

  chunk = valid_chunk();
  chunk.source_id = "wrong";
  EXPECT_EQ(
    validator_.validate_and_accept(chunk, 0.0).fault,
    xm::StreamFault::kWrongSource);

  chunk = valid_chunk(1);
  EXPECT_EQ(
    validator_.validate_and_accept(chunk, 0.0).fault,
    xm::StreamFault::kSequence);

  chunk = valid_chunk();
  chunk.age_s = 0.26;
  EXPECT_EQ(validator_.validate_and_accept(chunk, 0.0).fault, xm::StreamFault::kStale);

  chunk = valid_chunk();
  chunk.age_s = -0.06;
  EXPECT_EQ(
    validator_.validate_and_accept(chunk, 0.0).fault,
    xm::StreamFault::kFutureDated);

  chunk = valid_chunk();
  chunk.sample_period_s = 0.2;
  EXPECT_EQ(
    validator_.validate_and_accept(chunk, 0.0).fault,
    xm::StreamFault::kSamplePeriod);
  EXPECT_EQ(validator_.next_sequence(), 0U);
}

TEST_F(StreamingValidatorTest, RejectsLayoutDimensionsAndNonFiniteValues)
{
  auto chunk = valid_chunk();
  std::swap(chunk.joint_names[0], chunk.joint_names[1]);
  EXPECT_EQ(
    validator_.validate_and_accept(chunk, 0.0).fault,
    xm::StreamFault::kJointLayout);

  chunk = valid_chunk();
  chunk.positions.pop_back();
  EXPECT_EQ(
    validator_.validate_and_accept(chunk, 0.0).fault,
    xm::StreamFault::kDimensions);

  chunk = valid_chunk();
  chunk.positions[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    validator_.validate_and_accept(chunk, 0.0).fault,
    xm::StreamFault::kNonFinite);
}

TEST_F(StreamingValidatorTest, RejectsPositionAndQueueLimits)
{
  auto chunk = valid_chunk();
  chunk.positions[0] = 1.01;
  EXPECT_EQ(
    validator_.validate_and_accept(chunk, 0.0).fault,
    xm::StreamFault::kPositionLimit);

  chunk = valid_chunk();
  EXPECT_EQ(
    validator_.validate_and_accept(chunk, 0.45).fault,
    xm::StreamFault::kQueueHorizon);
}

TEST(StreamingValidatorSemanticsTest, AcceptsVerifiedQuarterSecondPositionSetpointClose)
{
  auto servo_config = config();
  servo_config.joints[1] = {
    "gripper", 0.0, 1.65,
    xm::JointCommandSemantics::kPositionSetpoint};
  xm::StreamingValidator validator(servo_config);
  ASSERT_TRUE(validator.begin("session-1", "act-policy", {0.0, 1.64}, {0.0, 0.0}));

  auto chunk = valid_chunk();
  chunk.sample_period_s = 1.0 / 30.0;
  chunk.sample_count = 8;
  chunk.final_chunk = true;
  chunk.positions = {
    0.0, 1.435,
    0.0, 1.230,
    0.0, 1.025,
    0.0, 0.820,
    0.0, 0.615,
    0.0, 0.410,
    0.0, 0.205,
    0.0, 0.000,
  };
  const auto accepted = validator.validate_and_accept(chunk, 0.0);
  EXPECT_TRUE(accepted) << accepted.message;
  EXPECT_TRUE(validator.final_received());
}

TEST_F(StreamingValidatorTest, RejectionIsTransactionalAndResetClosesSession)
{
  const auto before = validator_.last_positions();
  auto chunk = valid_chunk();
  chunk.positions[3] = 3.0;
  EXPECT_FALSE(validator_.validate_and_accept(chunk, 0.0));
  EXPECT_EQ(validator_.last_positions(), before);
  EXPECT_EQ(validator_.next_sequence(), 0U);

  validator_.reset();
  EXPECT_FALSE(validator_.active());
  EXPECT_EQ(
    validator_.validate_and_accept(valid_chunk(), 0.0).fault,
    xm::StreamFault::kNoActiveSession);
}
