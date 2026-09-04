#pragma once

#include <string>
#include <vector>

#include "xlerobot_manipulation/streaming_validator.hpp"

namespace xlerobot_manipulation
{

struct PolicyReplay
{
  std::string format;
  std::string provenance_kind;
  std::string provenance_note;
  std::string session_id;
  std::string source_id;
  std::vector<std::string> joint_names;
  std::vector<double> start_positions;
  std::vector<double> start_velocities;
  std::vector<PolicyChunk> chunks;
};

PolicyReplay load_policy_replay(const std::string & path);

struct ReplayValidation
{
  bool accepted{false};
  size_t accepted_chunks{0};
  StreamFault fault{StreamFault::kNone};
  std::string message;
};

ReplayValidation validate_policy_replay(
  const PolicyReplay & replay,
  const StreamSafetyConfig & config);

}  // namespace xlerobot_manipulation
