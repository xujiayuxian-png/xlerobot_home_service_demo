// Copyright 2026 Lisa
#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace xlerobot_feetech
{
struct ServoCalibrationSpec
{
  std::string group;
  std::string name;
  uint8_t id;
  int direction;
  double lower;
  double upper;
  std::string key() const {return group + "." + name;}
};

struct ServoCalibrationCapture
{
  std::optional<int> position;
  std::optional<int> zero;
  std::optional<int> reference_zero;
  std::optional<int> raw_min;
  std::optional<int> raw_max;
  std::optional<int> last_sample;
  bool online{false};
  bool range_captured{false};
  std::string zero_source;
  std::string fault;
  std::string read_error;
};

// ROS/device-free state machine. The caller owns reads and explicit torque release.
class ServoCalibrationSession
{
public:
  using Positions = std::vector<std::optional<int>>;
  ServoCalibrationSession(std::string unit_id, std::string bus_identity, bool leader_only = false);
  static const std::vector<ServoCalibrationSpec> & specs();
  const std::vector<ServoCalibrationSpec> & selected_specs() const {return specs_;}
  std::vector<size_t> group(const std::string & name) const;
  const std::vector<ServoCalibrationCapture> & captures() const {return captures_;}
  const std::string & phase() const {return phase_;}
  const std::string & active_group() const {return active_group_;}
  bool recording() const {return phase_ == "RANGE_RECORDING";}
  bool finalized() const {return phase_ == "FINALIZED";}
  double coverage(size_t index) const;
  std::string message(size_t index) const;
  std::vector<std::string> completed_groups() const;
  void check_group_change(const std::string & name) const;
  void observe(const std::string & name, const Positions & positions);
  void capture_zero(const std::string & name, const Positions & positions);
  void use_existing_zero(const std::string & name);
  void start_range(const std::string & name);
  void finish_range(const std::string & name);
  void pause_range(const std::string & name);
  void reset_group(const std::string & name);
  void load_reference(const std::filesystem::path & file, const std::string & version);
  void save(const std::filesystem::path & file) const;
  void restore(const std::filesystem::path & file);
  void recover_finalized_result(const std::filesystem::path & file);
  void finalize(const std::filesystem::path & file);

private:
  void ensure_mutable() const;
  void validate_complete() const;
  void set_zeros(const std::string & name, const Positions & positions, const std::string & source);
  std::string unit_id_;
  bool leader_only_;
  std::vector<ServoCalibrationSpec> specs_;
  std::vector<std::string> groups_;
  std::string bus_identity_;
  std::string reference_version_;
  std::string phase_{"IDLE"};
  std::string active_group_;
  std::vector<ServoCalibrationCapture> captures_;
};
}  // namespace xlerobot_feetech
