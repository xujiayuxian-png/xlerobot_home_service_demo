// Copyright 2026 Lisa
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "xlerobot_feetech/servo_calibration_session.hpp"

namespace
{
using Session = xlerobot_feetech::ServoCalibrationSession;

class ServoSessionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char pattern[] = "/tmp/xlerobot-servo-session-XXXXXX";
    const auto path = mkdtemp(pattern);
    ASSERT_NE(path, nullptr);
    folder = path;
  }
  void TearDown() override {std::filesystem::remove_all(folder);}

  Session::Positions frame(const std::string & group, int value) const
  {
    return Session::Positions(session.group(group).size(), value);
  }

  void start(const std::string & group)
  {
    session.capture_zero(group, frame(group, 2048));
    session.start_range(group);
  }

  void sweep(const std::string & group)
  {
    session.observe(group, frame(group, 500));
    session.observe(group, frame(group, 2048));
    session.observe(group, frame(group, 3596));
  }

  void complete(const std::string & group)
  {
    start(group);
    sweep(group);
    session.finish_range(group);
  }

  void all_groups()
  {
    complete("right_arm");
    complete("left_arm");
    complete("head");
  }

  void write_yaml(const std::filesystem::path & file, const YAML::Node & yaml)
  {
    std::ofstream stream(file);
    stream << yaml;
  }

  std::filesystem::path folder;
  Session session{"test-unit", "right-bus|left-bus"};
};

TEST_F(ServoSessionTest, OneWholeArmCaptureRecordsAllSixAndLeavesOthersUntouched)
{
  start("right_arm");
  EXPECT_EQ(session.captures().size(), 14U);
  EXPECT_EQ(session.active_group(), "right_arm");
  sweep("right_arm");
  session.finish_range("right_arm");
  EXPECT_EQ(session.completed_groups(), std::vector<std::string>{"right_arm"});
  for (const auto i : session.group("right_arm")) {
    EXPECT_EQ(session.captures()[i].zero, 2048);
    EXPECT_EQ(session.captures()[i].raw_min, 500);
    EXPECT_EQ(session.captures()[i].raw_max, 3596);
    EXPECT_TRUE(session.captures()[i].range_captured);
    EXPECT_GE(session.coverage(i), 0.60);
  }
  for (const auto i : session.group("left_arm")) {EXPECT_FALSE(session.captures()[i].zero);}
}

TEST_F(ServoSessionTest, LeaderIsAnIndependentSixJointSessionAndResult)
{
  Session leader("unit", "leader-port", true);
  EXPECT_EQ(leader.selected_specs().size(), 6U);
  EXPECT_THROW(leader.group("right_arm"), std::runtime_error);
  EXPECT_THROW(leader.group("head"), std::runtime_error);
  leader.capture_zero("leader", Session::Positions(6, 2048));
  leader.start_range("leader");
  for (int value : {500, 2048, 3596}) {leader.observe("leader", Session::Positions(6, value));}
  leader.finish_range("leader");
  const auto result = folder / "leader.yaml";
  leader.finalize(result);
  const auto document = YAML::LoadFile(result.string());
  EXPECT_EQ(document["attachment"].as<std::string>(), "right_leader");
  EXPECT_EQ(document["joints"].size(), 6U);
  EXPECT_FALSE(document["right_arm"]);
  leader.save(folder / "leader-session.yaml");
  Session restored("unit", "leader-port", true);
  restored.restore(folder / "leader-session.yaml");
  restored.recover_finalized_result(result);
  EXPECT_TRUE(restored.finalized());
  Session fresh("unit", "leader-port", true);
  fresh.load_reference(result, "local-result");
  EXPECT_EQ(fresh.captures()[0].reference_zero.value(), 2048);
}

TEST_F(ServoSessionTest, InsufficientFinishKeepsRecordingAndNamesDeficientJoints)
{
  start("right_arm");
  session.observe("right_arm", frame("right_arm", 2100));
  try {
    session.finish_range("right_arm");
    FAIL() << "an insufficient range must fail";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("shoulder_pan"), std::string::npos);
    EXPECT_NE(std::string(error.what()).find("progress kept"), std::string::npos);
  }
  EXPECT_TRUE(session.recording());
  EXPECT_EQ(session.captures()[0].raw_max, 2100);
  sweep("right_arm");
  EXPECT_NO_THROW(session.finish_range("right_arm"));
}

TEST_F(ServoSessionTest, AllRowsRequiredInsteadOfOnlyTheSelectedOrLastJoint)
{
  start("right_arm");
  auto positions = frame("right_arm", 500);
  positions[4] = 2048;
  session.observe("right_arm", positions);
  session.observe("right_arm", frame("right_arm", 2048));
  positions = frame("right_arm", 3596);
  positions[4] = 2048;
  session.observe("right_arm", positions);
  EXPECT_THROW(session.finish_range("right_arm"), std::runtime_error);
  EXPECT_TRUE(session.recording());
  EXPECT_TRUE(session.completed_groups().empty());
}

TEST_F(ServoSessionTest, ZeroCaptureIsAtomicAndRecaptureInvalidatesWholeGroupRanges)
{
  complete("right_arm");
  auto incomplete = frame("right_arm", 1900);
  incomplete[4] = std::nullopt;
  EXPECT_THROW(session.capture_zero("right_arm", incomplete), std::runtime_error);
  for (const auto i : session.group("right_arm")) {
    EXPECT_EQ(session.captures()[i].zero, 2048);
    EXPECT_TRUE(session.captures()[i].range_captured);
  }
  incomplete[4] = 4096;
  EXPECT_THROW(session.capture_zero("right_arm", incomplete), std::runtime_error);
  session.capture_zero("right_arm", frame("right_arm", 0));
  for (const auto i : session.group("right_arm")) {
    EXPECT_EQ(session.captures()[i].zero, 0);
    EXPECT_FALSE(session.captures()[i].raw_min);
    EXPECT_FALSE(session.captures()[i].range_captured);
    EXPECT_EQ(session.captures()[i].zero_source, "measured");
  }
}

TEST_F(ServoSessionTest, PauseResumeAndRestartPreserveProgressButNeverAutoResume)
{
  start("right_arm");
  session.observe("right_arm", frame("right_arm", 1000));
  session.pause_range("right_arm");
  session.observe("right_arm", frame("right_arm", 900));
  EXPECT_EQ(session.captures()[0].raw_min, 1000);
  session.start_range("right_arm");
  EXPECT_EQ(session.captures()[0].raw_min, 900);
  session.save(folder / "session.yaml");
  Session restarted("test-unit", "right-bus|left-bus");
  restarted.restore(folder / "session.yaml");
  EXPECT_EQ(restarted.phase(), "PAUSED");
  EXPECT_EQ(restarted.active_group(), "right_arm");
  EXPECT_EQ(restarted.captures()[0].raw_min, 900);
  EXPECT_FALSE(restarted.captures()[0].online);
  EXPECT_FALSE(restarted.captures()[0].position);
  EXPECT_THROW(restarted.start_range("right_arm"), std::runtime_error);
  restarted.observe("right_arm", frame("right_arm", 1200));
  restarted.start_range("right_arm");
  EXPECT_TRUE(restarted.recording());
  EXPECT_EQ(restarted.captures()[0].raw_min, 900);
}

TEST_F(ServoSessionTest, ResetGroupIsIsolatedAndCannotInterruptAnotherRecording)
{
  complete("right_arm");
  start("left_arm");
  EXPECT_THROW(session.reset_group("right_arm"), std::runtime_error);
  EXPECT_THROW(session.capture_zero("head", frame("head", 2048)), std::runtime_error);
  session.pause_range("left_arm");
  session.reset_group("left_arm");
  EXPECT_EQ(session.completed_groups(), std::vector<std::string>{"right_arm"});
  for (const auto i : session.group("left_arm")) {EXPECT_FALSE(session.captures()[i].zero);}
  EXPECT_EQ(session.phase(), "IDLE");
  EXPECT_TRUE(session.active_group().empty());
}

TEST_F(ServoSessionTest, ResetOtherGroupPreservesPausedGroupAndItsPhase)
{
  complete("right_arm");
  start("left_arm");
  session.pause_range("left_arm");
  session.reset_group("right_arm");
  EXPECT_EQ(session.phase(), "PAUSED");
  EXPECT_EQ(session.active_group(), "left_arm");
  EXPECT_EQ(session.captures()[6].zero, 2048);
}

TEST_F(ServoSessionTest, ReadFailurePreventsStaleFinishAndCanRecover)
{
  start("right_arm");
  sweep("right_arm");
  auto sample = frame("right_arm", 3596);
  sample[2] = std::nullopt;
  session.observe("right_arm", sample);
  EXPECT_FALSE(session.captures()[2].online);
  EXPECT_NE(session.message(2).find("read failed"), std::string::npos);
  EXPECT_THROW(session.finish_range("right_arm"), std::runtime_error);
  session.observe("right_arm", frame("right_arm", 3596));
  EXPECT_NO_THROW(session.finish_range("right_arm"));
}

TEST_F(ServoSessionTest, EncoderWrapCannotBecomeAnApparentFullRange)
{
  session.capture_zero("right_arm", frame("right_arm", 100));
  session.start_range("right_arm");
  session.observe("right_arm", frame("right_arm", 4090));
  EXPECT_EQ(session.captures()[0].raw_min, 100);
  EXPECT_EQ(session.captures()[0].raw_max, 100);
  EXPECT_NE(session.message(0).find("wrapped"), std::string::npos);
  session.observe("right_arm", frame("right_arm", 4080));
  EXPECT_THROW(session.finish_range("right_arm"), std::runtime_error);
  session.save(folder / "session.yaml");
  Session restarted("test-unit", "right-bus|left-bus");
  restarted.restore(folder / "session.yaml");
  EXPECT_NE(restarted.message(0).find("wrapped"), std::string::npos);
  EXPECT_THROW(restarted.start_range("right_arm"), std::runtime_error);
  restarted.reset_group("right_arm");
  EXPECT_TRUE(restarted.captures()[0].fault.empty());
}

TEST_F(ServoSessionTest, InvalidEncoderFrameBlocksPassEvenAfterLaterValidReads)
{
  start("right_arm");
  sweep("right_arm");
  auto sample = frame("right_arm", 3596);
  sample[1] = 6000;
  session.observe("right_arm", sample);
  EXPECT_FALSE(session.captures()[1].online);
  session.observe("right_arm", frame("right_arm", 3596));
  EXPECT_THROW(session.finish_range("right_arm"), std::runtime_error);
  EXPECT_NE(session.message(1).find("invalid encoder"), std::string::npos);
}

TEST_F(ServoSessionTest, ARangeMustIncludeTheCapturedZero)
{
  session.capture_zero("head", frame("head", 100));
  session.observe("head", frame("head", 1000));
  session.start_range("head");
  session.observe("head", frame("head", 2900));
  EXPECT_GE(session.coverage(12), 0.60);
  EXPECT_THROW(session.finish_range("head"), std::runtime_error);
  EXPECT_NE(session.message(12).find("zero pose"), std::string::npos);
}

TEST_F(ServoSessionTest, FinalizeRequiresFourteenAndCannotOverwriteAnExistingResult)
{
  complete("right_arm");
  complete("left_arm");
  EXPECT_THROW(session.finalize(folder / "result.yaml"), std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(folder / "result.yaml"));
  complete("head");
  {std::ofstream stream(folder / "result.yaml"); stream << "keep-existing-result\n";}
  EXPECT_THROW(session.finalize(folder / "result.yaml"), std::runtime_error);
  std::ifstream stream(folder / "result.yaml");
  std::string text;
  std::getline(stream, text);
  EXPECT_EQ(text, "keep-existing-result");
  EXPECT_FALSE(session.finalized());
  session.finalize(folder / "new-result.yaml");
  EXPECT_TRUE(session.finalized());
  EXPECT_THROW(session.reset_group("right_arm"), std::runtime_error);
  const auto node = YAML::LoadFile((folder / "new-result.yaml").string());
  EXPECT_EQ(node["schema"].as<std::string>(), "xlerobot_servo_calibration/v1");
  for (const auto & spec : Session::specs()) {
    EXPECT_EQ(node[spec.group]["joints"][spec.name]["offset"].as<int>(), 2048);
  }
}

TEST_F(ServoSessionTest, RejectsMismatchedUnitSchemaLayoutAndCorruptValuesWithoutMutation)
{
  start("right_arm");
  session.save(folder / "session.yaml");
  const auto good = YAML::LoadFile((folder / "session.yaml").string());
  for (const auto key : {"schema", "unit_id", "bus_identity"}) {
    auto bad = YAML::Clone(good);
    bad[key] = "incorrect";
    write_yaml(folder / "bad.yaml", bad);
    EXPECT_THROW(session.restore(folder / "bad.yaml"), std::runtime_error);
    EXPECT_TRUE(session.recording());
  }
  for (const auto key : {"zero", "raw_min", "last_sample"}) {
    auto bad = YAML::Clone(good);
    bad["joints"]["right_arm.shoulder_pan"][key] = 9000;
    write_yaml(folder / "bad.yaml", bad);
    EXPECT_THROW(session.restore(folder / "bad.yaml"), std::runtime_error);
  }
  auto bad = YAML::Clone(good);
  bad["joints"]["right_arm.shoulder_pan"]["limit_min"] = ".nan";
  write_yaml(folder / "bad.yaml", bad);
  EXPECT_THROW(session.restore(folder / "bad.yaml"), std::exception);
  bad = YAML::Clone(good);
  bad["joints"].remove("head.tilt");
  write_yaml(folder / "bad.yaml", bad);
  EXPECT_THROW(session.restore(folder / "bad.yaml"), std::runtime_error);
  EXPECT_EQ(session.captures()[0].zero, 2048);
}

TEST_F(ServoSessionTest, ReusingVerifiedZerosIsExplicitAndDoesNotReuseRanges)
{
  all_groups();
  session.finalize(folder / "reference.yaml");
  Session fresh("test-unit", "right-bus|left-bus");
  fresh.load_reference(folder / "reference.yaml", "test-reference-v1");
  EXPECT_FALSE(fresh.captures()[0].zero);
  EXPECT_EQ(fresh.captures()[0].reference_zero, 2048);
  fresh.use_existing_zero("right_arm");
  for (const auto i : fresh.group("right_arm")) {
    EXPECT_EQ(fresh.captures()[i].zero, 2048);
    EXPECT_EQ(fresh.captures()[i].zero_source, "existing:test-reference-v1");
    EXPECT_FALSE(fresh.captures()[i].raw_min);
    EXPECT_FALSE(fresh.captures()[i].range_captured);
    EXPECT_FALSE(fresh.captures()[i].online);
  }
  EXPECT_FALSE(fresh.captures()[6].zero);
  fresh.save(folder / "session.yaml");
  Session restarted("test-unit", "right-bus|left-bus");
  restarted.restore(folder / "session.yaml");
  EXPECT_EQ(restarted.captures()[0].zero_source, "existing:test-reference-v1");
}

TEST_F(ServoSessionTest, BadExistingZeroFileCannotPartiallySeedAnyRows)
{
  all_groups();
  session.finalize(folder / "reference.yaml");
  auto bad = YAML::LoadFile((folder / "reference.yaml").string());
  bad["head"]["joints"]["tilt"]["servo_id"] = 9;
  write_yaml(folder / "bad.yaml", bad);
  Session fresh("test-unit", "right-bus|left-bus");
  EXPECT_THROW(fresh.load_reference(folder / "bad.yaml", "bad"), std::runtime_error);
  EXPECT_FALSE(fresh.captures()[0].reference_zero);
  EXPECT_THROW(fresh.use_existing_zero("right_arm"), std::runtime_error);
}

TEST_F(ServoSessionTest, CompletedGroupCanBeExtendedWithoutDiscardingItsRange)
{
  complete("right_arm");
  session.observe("right_arm", frame("right_arm", 3500));
  session.start_range("right_arm");
  EXPECT_EQ(session.captures()[0].raw_min, 500);
  EXPECT_EQ(session.captures()[0].raw_max, 3596);
  EXPECT_TRUE(session.captures()[0].range_captured);
  session.observe("right_arm", frame("right_arm", 3600));
  session.finish_range("right_arm");
  EXPECT_EQ(session.captures()[0].raw_max, 3600);
}

TEST_F(ServoSessionTest, RestartRecoversPublishedResultAfterFinalSessionSaveWasInterrupted)
{
  all_groups();
  session.save(folder / "session.yaml");
  session.finalize(folder / "result.yaml");
  Session restarted("test-unit", "right-bus|left-bus");
  restarted.restore(folder / "session.yaml");
  EXPECT_FALSE(restarted.finalized());
  restarted.recover_finalized_result(folder / "result.yaml");
  EXPECT_TRUE(restarted.finalized());
  auto changed = YAML::LoadFile((folder / "result.yaml").string());
  changed["right_arm"]["joints"]["shoulder_pan"]["offset"] = 2047;
  write_yaml(folder / "changed.yaml", changed);
  Session another("test-unit", "right-bus|left-bus");
  another.restore(folder / "session.yaml");
  EXPECT_THROW(another.recover_finalized_result(folder / "changed.yaml"), std::runtime_error);
  EXPECT_FALSE(another.finalized());
}
}  // namespace
