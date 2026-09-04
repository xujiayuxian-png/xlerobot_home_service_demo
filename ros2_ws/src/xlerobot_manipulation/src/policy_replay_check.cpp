#include <exception>
#include <iostream>
#include <string>

#include "xlerobot_manipulation/policy_replay.hpp"

namespace xm = xlerobot_manipulation;

namespace
{

xm::StreamSafetyConfig reference_config()
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

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: policy_replay_check REPLAY.yaml\n";
    return 2;
  }
  try {
    const auto replay = xm::load_policy_replay(argv[1]);
    const auto result = xm::validate_policy_replay(replay, reference_config());
    std::cout << "provenance=" << replay.provenance_kind <<
      " accepted_chunks=" << result.accepted_chunks <<
      " result=" << result.message << '\n';
    return result.accepted ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
