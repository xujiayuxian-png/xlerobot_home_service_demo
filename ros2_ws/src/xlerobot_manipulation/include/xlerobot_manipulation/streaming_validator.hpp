#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace xlerobot_manipulation
{

enum class JointCommandSemantics
{
  kTrajectory,
  kPositionSetpoint,
};

struct JointEnvelope
{
  std::string name;
  double lower_position{0.0};
  double upper_position{0.0};
  JointCommandSemantics command_semantics{JointCommandSemantics::kTrajectory};
};

struct StreamSafetyConfig
{
  std::vector<JointEnvelope> joints;
  double min_sample_period_s{0.01};
  double max_sample_period_s{0.1};
  double max_command_age_s{0.25};
  double max_future_skew_s{0.05};
  double max_queue_horizon_s{2.0};
};

struct PolicyChunk
{
  std::string session_id;
  std::string source_id;
  uint64_t sequence{0};
  double age_s{0.0};
  double sample_period_s{0.0};
  bool final_chunk{false};
  std::vector<std::string> joint_names;
  uint32_t sample_count{0};
  std::vector<double> positions;
};

enum class StreamFault
{
  kNone,
  kInvalidConfig,
  kSessionAlreadyActive,
  kNoActiveSession,
  kWrongSession,
  kWrongSource,
  kSequence,
  kStale,
  kFutureDated,
  kSamplePeriod,
  kJointLayout,
  kDimensions,
  kNonFinite,
  kPositionLimit,
  kQueueHorizon,
  kAlreadyFinal,
};

struct ValidationResult
{
  StreamFault fault{StreamFault::kNone};
  std::string message;

  explicit operator bool() const {return fault == StreamFault::kNone;}
};

class StreamingValidator
{
public:
  explicit StreamingValidator(StreamSafetyConfig config);

  ValidationResult begin(
    const std::string & session_id,
    const std::string & source_id,
    const std::vector<double> & measured_positions,
    const std::vector<double> & measured_velocities);
  ValidationResult validate_and_accept(const PolicyChunk & chunk, double queued_horizon_s);
  void reset();

  bool active() const {return active_;}
  bool final_received() const {return final_received_;}
  uint64_t next_sequence() const {return next_sequence_;}
  const std::vector<double> & last_positions() const {return last_positions_;}

private:
  ValidationResult reject(StreamFault fault, const std::string & message) const;
  ValidationResult validate_config() const;

  StreamSafetyConfig config_;
  std::string session_id_;
  std::string source_id_;
  uint64_t next_sequence_{0};
  bool active_{false};
  bool final_received_{false};
  std::vector<double> last_positions_;
};

}  // namespace xlerobot_manipulation
