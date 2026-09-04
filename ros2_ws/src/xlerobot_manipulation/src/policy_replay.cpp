#include "xlerobot_manipulation/policy_replay.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace xlerobot_manipulation
{

namespace
{

template<typename T>
T required(const YAML::Node & node, const std::string & key)
{
  if (!node[key]) {
    throw std::runtime_error("policy replay is missing required key: " + key);
  }
  return node[key].as<T>();
}

}  // namespace

PolicyReplay load_policy_replay(const std::string & path)
{
  const auto root = YAML::LoadFile(path);
  PolicyReplay replay;
  replay.format = required<std::string>(root, "format");
  if (replay.format != "xlerobot_policy_replay/v1") {
    throw std::runtime_error("unsupported policy replay format: " + replay.format);
  }
  const auto provenance = root["provenance"];
  replay.provenance_kind = required<std::string>(provenance, "kind");
  replay.provenance_note = required<std::string>(provenance, "note");
  replay.session_id = required<std::string>(root, "session_id");
  replay.source_id = required<std::string>(root, "source_id");
  replay.joint_names = required<std::vector<std::string>>(root, "joint_names");
  replay.start_positions = required<std::vector<double>>(root, "start_positions");
  replay.start_velocities = required<std::vector<double>>(root, "start_velocities");
  const auto chunks = root["chunks"];
  if (!chunks || !chunks.IsSequence() || chunks.size() == 0) {
    throw std::runtime_error("policy replay chunks must be a non-empty sequence");
  }
  for (const auto & item : chunks) {
    PolicyChunk chunk;
    chunk.session_id = replay.session_id;
    chunk.source_id = replay.source_id;
    chunk.sequence = required<uint64_t>(item, "sequence");
    chunk.age_s = required<double>(item, "age_s");
    chunk.sample_period_s = required<double>(item, "sample_period_s");
    chunk.final_chunk = required<bool>(item, "final_chunk");
    chunk.joint_names = replay.joint_names;
    const auto rows = item["positions"];
    if (!rows || !rows.IsSequence() || rows.size() == 0) {
      throw std::runtime_error("policy replay chunk positions must be non-empty rows");
    }
    chunk.sample_count = static_cast<uint32_t>(rows.size());
    for (const auto & row : rows) {
      const auto values = row.as<std::vector<double>>();
      chunk.positions.insert(chunk.positions.end(), values.begin(), values.end());
    }
    replay.chunks.push_back(std::move(chunk));
  }
  return replay;
}

ReplayValidation validate_policy_replay(
  const PolicyReplay & replay,
  const StreamSafetyConfig & config)
{
  StreamingValidator validator(config);
  const auto begin = validator.begin(
    replay.session_id, replay.source_id, replay.start_positions, replay.start_velocities);
  if (!begin) {
    return {false, 0, begin.fault, begin.message};
  }
  size_t accepted = 0;
  for (const auto & chunk : replay.chunks) {
    const auto result = validator.validate_and_accept(chunk, 0.0);
    if (!result) {
      return {false, accepted, result.fault, result.message};
    }
    ++accepted;
  }
  if (!validator.final_received()) {
    return {false, accepted, StreamFault::kDimensions, "replay has no final chunk"};
  }
  return {true, accepted, StreamFault::kNone, "all replay chunks accepted"};
}

}  // namespace xlerobot_manipulation
