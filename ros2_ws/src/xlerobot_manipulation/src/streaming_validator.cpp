#include "xlerobot_manipulation/streaming_validator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace xlerobot_manipulation
{

namespace
{

constexpr double kTolerance = 1.0e-9;

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

std::string joint_error(const std::string & prefix, const std::string & joint)
{
  return prefix + " for joint " + joint;
}

}  // namespace

StreamingValidator::StreamingValidator(StreamSafetyConfig config)
: config_(std::move(config))
{
}

ValidationResult StreamingValidator::begin(
  const std::string & session_id,
  const std::string & source_id,
  const std::vector<double> & measured_positions,
  const std::vector<double> & measured_velocities)
{
  if (active_) {
    return reject(StreamFault::kSessionAlreadyActive, "a stream session is already active");
  }
  const auto config_result = validate_config();
  if (!config_result) {
    return config_result;
  }
  if (session_id.empty() || source_id.empty()) {
    return reject(StreamFault::kInvalidConfig, "session and source IDs must be non-empty");
  }
  if (measured_positions.size() != config_.joints.size() ||
    measured_velocities.size() != config_.joints.size())
  {
    return reject(StreamFault::kDimensions, "measured start state dimensions do not match");
  }
  for (size_t index = 0; index < measured_positions.size(); ++index) {
    const auto & joint = config_.joints[index];
    const double position = measured_positions[index];
    if (!std::isfinite(position)) {
      return reject(StreamFault::kNonFinite,
          joint_error("non-finite measured position", joint.name));
    }
    if (!std::isfinite(measured_velocities[index])) {
      return reject(StreamFault::kNonFinite,
          joint_error("non-finite measured velocity", joint.name));
    }
    if (position < joint.lower_position - kTolerance ||
      position > joint.upper_position + kTolerance)
    {
      return reject(
        StreamFault::kPositionLimit, joint_error("measured position outside envelope", joint.name));
    }
  }

  session_id_ = session_id;
  source_id_ = source_id;
  next_sequence_ = 0;
  active_ = true;
  final_received_ = false;
  last_positions_ = measured_positions;
  return {};
}

ValidationResult StreamingValidator::validate_and_accept(
  const PolicyChunk & chunk, double queued_horizon_s)
{
  if (!active_) {
    return reject(StreamFault::kNoActiveSession, "no stream session is active");
  }
  if (final_received_) {
    return reject(StreamFault::kAlreadyFinal, "the final chunk was already accepted");
  }
  if (chunk.session_id != session_id_) {
    return reject(StreamFault::kWrongSession, "chunk session ID does not match");
  }
  if (chunk.source_id != source_id_) {
    return reject(StreamFault::kWrongSource, "chunk source ID does not match");
  }
  if (chunk.sequence != next_sequence_) {
    return reject(StreamFault::kSequence, "chunk sequence is not the exact next value");
  }
  if (!std::isfinite(chunk.age_s)) {
    return reject(StreamFault::kNonFinite, "chunk age is non-finite");
  }
  if (chunk.age_s > config_.max_command_age_s + kTolerance) {
    return reject(StreamFault::kStale, "chunk generation time is stale");
  }
  if (chunk.age_s < -config_.max_future_skew_s - kTolerance) {
    return reject(StreamFault::kFutureDated, "chunk generation time is too far in the future");
  }
  if (!std::isfinite(chunk.sample_period_s) ||
    chunk.sample_period_s < config_.min_sample_period_s - kTolerance ||
    chunk.sample_period_s > config_.max_sample_period_s + kTolerance)
  {
    return reject(StreamFault::kSamplePeriod, "sample period is outside the configured range");
  }
  if (!std::isfinite(queued_horizon_s) || queued_horizon_s < 0.0) {
    return reject(StreamFault::kQueueHorizon, "current queue horizon is invalid");
  }

  std::vector<std::string> expected_names;
  expected_names.reserve(config_.joints.size());
  for (const auto & joint : config_.joints) {
    expected_names.push_back(joint.name);
  }
  if (chunk.joint_names != expected_names) {
    return reject(StreamFault::kJointLayout, "joint names or ordering do not match the session");
  }
  if (chunk.sample_count == 0) {
    return reject(StreamFault::kDimensions, "a chunk must contain at least one sample");
  }
  const size_t joint_count = config_.joints.size();
  const size_t expected_values = static_cast<size_t>(chunk.sample_count) * joint_count;
  if (chunk.positions.size() != expected_values) {
    return reject(StreamFault::kDimensions, "row-major position dimensions do not match");
  }
  const double new_horizon = queued_horizon_s +
    static_cast<double>(chunk.sample_count) * chunk.sample_period_s;
  if (new_horizon > config_.max_queue_horizon_s + kTolerance) {
    return reject(StreamFault::kQueueHorizon, "accepted samples would exceed queue horizon");
  }

  std::vector<double> previous_positions = last_positions_;
  for (uint32_t sample = 0; sample < chunk.sample_count; ++sample) {
    for (size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
      const auto & envelope = config_.joints[joint_index];
      const double position = chunk.positions[sample * joint_count + joint_index];
      if (!std::isfinite(position)) {
        return reject(StreamFault::kNonFinite, joint_error("non-finite command", envelope.name));
      }
      if (position < envelope.lower_position - kTolerance ||
        position > envelope.upper_position + kTolerance)
      {
        return reject(
          StreamFault::kPositionLimit, joint_error("position limit exceeded", envelope.name));
      }
    }
    const auto offset = static_cast<size_t>(sample) * joint_count;
    previous_positions.assign(
      chunk.positions.begin() + static_cast<std::ptrdiff_t>(offset),
      chunk.positions.begin() + static_cast<std::ptrdiff_t>(offset + joint_count));
  }

  last_positions_ = std::move(previous_positions);
  ++next_sequence_;
  final_received_ = chunk.final_chunk;
  return {};
}

void StreamingValidator::reset()
{
  session_id_.clear();
  source_id_.clear();
  next_sequence_ = 0;
  active_ = false;
  final_received_ = false;
  last_positions_.clear();
}

ValidationResult StreamingValidator::reject(
  StreamFault fault, const std::string & message) const
{
  return {fault, message};
}

ValidationResult StreamingValidator::validate_config() const
{
  if (config_.joints.empty() || !finite_positive(config_.min_sample_period_s) ||
    !finite_positive(config_.max_sample_period_s) ||
    config_.min_sample_period_s > config_.max_sample_period_s ||
    !finite_positive(config_.max_command_age_s) ||
    !std::isfinite(config_.max_future_skew_s) || config_.max_future_skew_s < 0.0 ||
    !finite_positive(config_.max_queue_horizon_s))
  {
    return reject(StreamFault::kInvalidConfig, "stream safety configuration is invalid");
  }
  std::vector<std::string> names;
  names.reserve(config_.joints.size());
  for (const auto & joint : config_.joints) {
    if (joint.name.empty() || !std::isfinite(joint.lower_position) ||
      !std::isfinite(joint.upper_position) || joint.lower_position >= joint.upper_position ||
      std::find(names.begin(), names.end(), joint.name) != names.end())
    {
      return reject(StreamFault::kInvalidConfig, "joint safety envelope is invalid");
    }
    names.push_back(joint.name);
  }
  return {};
}

}  // namespace xlerobot_manipulation
