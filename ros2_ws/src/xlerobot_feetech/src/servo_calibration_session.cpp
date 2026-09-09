// Copyright 2026 Lisa
#include "xlerobot_feetech/servo_calibration_session.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace xlerobot_feetech
{
namespace
{
constexpr double kTicksPerRadian = 4096.0 / (2.0 * 3.14159265358979323846);
constexpr double kMinimumCoverage = 0.60;
const std::vector<std::string> kGroups{"right_arm", "left_arm", "head"};

bool valid_raw(int value) {return value >= 0 && value <= 4095;}

std::optional<int> optional_raw(const YAML::Node & node)
{
  const auto value = node.as<int>();
  if (value == -1) {return std::nullopt;}
  if (!valid_raw(value)) {throw std::runtime_error("session raw value outside 0..4095");}
  return value;
}

void atomic_yaml(const std::filesystem::path & file, const YAML::Node & node, bool immutable)
{
  if (!file.parent_path().empty()) {std::filesystem::create_directories(file.parent_path());}
  const std::filesystem::path temporary(file.string() + ".tmp");
  {
    std::ofstream stream(temporary, std::ios::trunc);
    if (!stream) {throw std::runtime_error("cannot create calibration file: " + temporary.string());}
    stream << node << '\n';
    stream.close();
    if (!stream) {throw std::runtime_error("failed to save calibration file: " + temporary.string());}
  }
  std::error_code error;
  if (immutable) {
    // Creating a hard link is atomic and refuses an existing result; never unlink
    // a previously valid result in response to a write/rename failure.
    std::filesystem::create_hard_link(temporary, file, error);
    if (!error) {std::filesystem::remove(temporary);}
  } else {
    std::filesystem::rename(temporary, file, error);
  }
  if (error) {throw std::runtime_error("cannot publish calibration file: " + error.message());}
}

void validate_spec(const YAML::Node & row, const ServoCalibrationSpec & spec)
{
  const auto lower = row["limit_min"].as<double>();
  const auto upper = row["limit_max"].as<double>();
  if (row["servo_id"].as<int>() != spec.id || row["direction"].as<int>() != spec.direction ||
    !std::isfinite(lower) || !std::isfinite(upper) ||
    std::abs(lower - spec.lower) > 1e-9 || std::abs(upper - spec.upper) > 1e-9)
  {
    throw std::runtime_error("calibration layout differs for " + spec.key());
  }
}
}  // namespace

const std::vector<ServoCalibrationSpec> & ServoCalibrationSession::specs()
{
  static const std::vector<ServoCalibrationSpec> rows = {
    {"right_arm", "shoulder_pan", 1, 1, -2.05, 2.05},
    {"right_arm", "shoulder_lift", 2, -1, -1.40, 1.85},
    {"right_arm", "elbow_flex", 3, 1, -1.65, 1.70},
    {"right_arm", "wrist_flex", 4, 1, -1.75, 1.75},
    {"right_arm", "wrist_roll", 5, -1, -3.09, 3.09},
    {"right_arm", "gripper", 6, 1, 0.0, 1.65},
    {"left_arm", "shoulder_pan", 1, 1, -2.05, 2.05},
    {"left_arm", "shoulder_lift", 2, -1, -1.45, 1.78},
    {"left_arm", "elbow_flex", 3, 1, -1.50, 1.70},
    {"left_arm", "wrist_flex", 4, 1, -1.75, 1.75},
    {"left_arm", "wrist_roll", 5, -1, -3.09, 3.09},
    {"left_arm", "gripper", 6, 1, 0.0, 1.65},
    {"head", "pan", 7, -1, -1.57, 1.57},
    {"head", "tilt", 8, 1, -0.76, 1.45},
  };
  return rows;
}

ServoCalibrationSession::ServoCalibrationSession(std::string unit_id, std::string bus_identity, bool leader_only)
: unit_id_(std::move(unit_id)), leader_only_(leader_only), specs_(specs()), groups_(kGroups),
  bus_identity_(std::move(bus_identity))
{
  if (leader_only_) {
    specs_.resize(6);
    for (auto & spec : specs_) {spec.group = "leader";}
    specs_[1].upper = 1.70;
    specs_[2].upper = 1.40;
    specs_[5].lower = 0.05;
    groups_ = {"leader"};
  }
  captures_.resize(specs_.size());
  if (unit_id_.empty()) {throw std::runtime_error("servo calibration requires unit_id");}
}

std::vector<size_t> ServoCalibrationSession::group(const std::string & name) const
{
  std::vector<size_t> indices;
  for (size_t i = 0; i < selected_specs().size(); ++i) {
    if (selected_specs()[i].group == name) {indices.push_back(i);}
  }
  if (indices.empty()) {throw std::runtime_error("group is not part of this calibration session");}
  return indices;
}

void ServoCalibrationSession::ensure_mutable() const
{
  if (finalized()) {throw std::runtime_error("session finalized; start a new capture with --fresh");}
}

void ServoCalibrationSession::check_group_change(const std::string & name) const
{
  group(name);
  ensure_mutable();
  if (recording() && active_group_ != name) {
    throw std::runtime_error("pause or finish " + active_group_ + " before changing groups");
  }
}

double ServoCalibrationSession::coverage(size_t index) const
{
  const auto & row = captures_.at(index);
  const auto & spec = selected_specs().at(index);
  if (!row.raw_min || !row.raw_max) {return 0.0;}
  return (*row.raw_max - *row.raw_min) / ((spec.upper - spec.lower) * kTicksPerRadian);
}

std::string ServoCalibrationSession::message(size_t index) const
{
  const auto & row = captures_.at(index);
  if (!row.fault.empty()) {return row.fault;}
  if (!row.read_error.empty()) {return row.read_error;}
  if (!row.online) {return "not read since startup";}
  if (!row.zero) {return "zero pose has not been captured";}
  if (!row.raw_min || !row.raw_max) {return "ready to record group range";}
  if (*row.zero < *row.raw_min || *row.zero > *row.raw_max) {
    return "include the zero pose in the recorded range";
  }
  if (coverage(index) < kMinimumCoverage) {
    return "move this joint further: coverage below 60%";
  }
  return row.range_captured ? "range captured" : "range sufficient";
}

std::vector<std::string> ServoCalibrationSession::completed_groups() const
{
  std::vector<std::string> complete;
  for (const auto & name : groups_) {
    const auto indices = group(name);
    if (std::all_of(indices.begin(), indices.end(), [&](size_t i) {
        return captures_[i].range_captured && captures_[i].fault.empty();
      })) {complete.push_back(name);}
  }
  return complete;
}

void ServoCalibrationSession::observe(const std::string & name, const Positions & positions)
{
  const auto indices = group(name);
  if (positions.size() != indices.size()) {throw std::runtime_error("wrong group sample size");}
  for (size_t j = 0; j < indices.size(); ++j) {
    auto & row = captures_[indices[j]];
    const auto position = positions[j];
    if (!position) {
      row.online = false;
      row.read_error = "servo read failed; check the cable and retry";
      continue;
    }
    if (!valid_raw(*position)) {
      row.online = false;
      row.read_error = "encoder value outside 0..4095";
      if (recording() && active_group_ == name) {
        row.fault = "invalid encoder range; reset this group and inspect the joint";
        row.range_captured = false;
      }
      continue;
    }
    row.position = *position;
    row.online = true;
    row.read_error.clear();
    if (!recording() || active_group_ != name) {continue;}
    if (row.last_sample && std::abs(*position - *row.last_sample) > 2048) {
      row.fault = "encoder wrapped or jumped >2048 ticks; reset this group; do not cross encoder zero";
      row.range_captured = false;
    }
    row.last_sample = *position;
    if (!row.fault.empty()) {continue;}
    row.raw_min = row.raw_min ? std::min(*row.raw_min, *position) : *position;
    row.raw_max = row.raw_max ? std::max(*row.raw_max, *position) : *position;
  }
}

void ServoCalibrationSession::set_zeros(
  const std::string & name, const Positions & positions, const std::string & source)
{
  check_group_change(name);
  if (recording()) {throw std::runtime_error("pause range recording before changing the zero pose");}
  const auto indices = group(name);
  if (positions.size() != indices.size() || std::any_of(
      positions.begin(), positions.end(), [](const auto & p) {return !p || !valid_raw(*p);}))
  {
    throw std::runtime_error("zero not saved: every joint must return a valid value in 0..4095");
  }
  for (size_t j = 0; j < indices.size(); ++j) {
    auto & row = captures_[indices[j]];
    row.zero = positions[j];
    row.zero_source = source;
    row.raw_min.reset();
    row.raw_max.reset();
    row.last_sample.reset();
    row.range_captured = false;
    row.fault.clear();
  }
  active_group_ = name;
  phase_ = "ZERO_CAPTURED";
}

void ServoCalibrationSession::capture_zero(const std::string & name, const Positions & positions)
{
  // Validate the entire frame before changing any zero/range.
  set_zeros(name, positions, "measured");
  observe(name, positions);
}

void ServoCalibrationSession::use_existing_zero(const std::string & name)
{
  Positions zeros;
  for (const auto index : group(name)) {zeros.push_back(captures_[index].reference_zero);}
  if (reference_version_.empty()) {throw std::runtime_error("no verified existing zero is available");}
  set_zeros(name, zeros, "existing:" + reference_version_);
}

void ServoCalibrationSession::start_range(const std::string & name)
{
  check_group_change(name);
  if (recording()) {throw std::runtime_error("this group is already recording");}
  for (const auto i : group(name)) {
    const auto & row = captures_[i];
    if (!row.zero) {throw std::runtime_error("capture or reuse the whole group's zero first");}
    if (!row.fault.empty()) {throw std::runtime_error(selected_specs()[i].key() + ": " + row.fault);}
    if (!row.online || !row.position) {throw std::runtime_error(selected_specs()[i].key() + ": read failed");}
  }
  active_group_ = name;
  phase_ = "RANGE_RECORDING";
  Positions positions;
  for (const auto i : group(name)) {positions.push_back(captures_[i].position);}
  observe(name, positions);
}

void ServoCalibrationSession::finish_range(const std::string & name)
{
  ensure_mutable();
  if (!recording() || active_group_ != name) {
    throw std::runtime_error("this group is not recording; resume it before finishing");
  }
  std::ostringstream missing;
  for (const auto i : group(name)) {
    const auto & row = captures_[i];
    if (!row.online || !row.zero || !row.raw_min || !row.raw_max || !row.fault.empty() ||
      *row.zero < *row.raw_min || *row.zero > *row.raw_max || coverage(i) < kMinimumCoverage)
    {
      if (missing.tellp() > 0) {missing << "; ";}
      missing << selected_specs()[i].key() << ": " << message(i);
    }
  }
  if (!missing.str().empty()) {
    throw std::runtime_error("still recording; progress kept — " + missing.str());
  }
  for (const auto i : group(name)) {captures_[i].range_captured = true;}
  phase_ = "RANGE_CAPTURED";
  active_group_.clear();
}

void ServoCalibrationSession::pause_range(const std::string & name)
{
  ensure_mutable();
  if (!recording() || active_group_ != name) {throw std::runtime_error("this group is not recording");}
  phase_ = "PAUSED";
}

void ServoCalibrationSession::reset_group(const std::string & name)
{
  check_group_change(name);
  for (const auto i : group(name)) {
    const auto reference = captures_[i].reference_zero;
    const auto position = captures_[i].position;
    const auto online = captures_[i].online;
    captures_[i] = ServoCalibrationCapture{};
    captures_[i].reference_zero = reference;
    captures_[i].position = position;
    captures_[i].online = online;
  }
  if (active_group_ == name || active_group_.empty()) {
    active_group_.clear();
    phase_ = "IDLE";
  }
}

void ServoCalibrationSession::load_reference(
  const std::filesystem::path & file, const std::string & version)
{
  if (version.empty()) {throw std::runtime_error("existing servo calibration requires a version");}
  auto node = YAML::LoadFile(file.string());
  if (leader_only_) {
    if (node["attachment"].as<std::string>("") != "right_leader") {
      throw std::runtime_error("expected a right_leader calibration, not Follower data");
    }
    YAML::Node wrapped;
    wrapped["leader"]["joints"] = node["joints"];
    node = wrapped;
  }
  Positions zeros;
  for (const auto & name : groups_) {
    if (!node[name]["joints"].IsMap() || node[name]["joints"].size() != group(name).size()) {
      throw std::runtime_error("existing servo calibration has the wrong joint set: " + name);
    }
  }
  for (const auto & spec : selected_specs()) {
    const auto row = node[spec.group]["joints"][spec.name];
    validate_spec(row, spec);
    const auto zero = row["offset"].as<int>();
    const auto low = row["raw_min"].as<int>();
    const auto high = row["raw_max"].as<int>();
    if (!valid_raw(zero) || !valid_raw(low) || !valid_raw(high) || low >= high ||
      zero < low || zero > high)
    {
      throw std::runtime_error("existing servo calibration has invalid raw limits: " + spec.key());
    }
    zeros.push_back(zero);
  }
  for (size_t i = 0; i < zeros.size(); ++i) {captures_[i].reference_zero = zeros[i];}
  reference_version_ = version;
}

void ServoCalibrationSession::save(const std::filesystem::path & file) const
{
  YAML::Node node;
  node["schema"] = "xlerobot_servo_session/v1";
  node["unit_id"] = unit_id_;
  node["bus_identity"] = bus_identity_;
  node["phase"] = phase_;
  node["active_group"] = active_group_;
  for (size_t i = 0; i < selected_specs().size(); ++i) {
    const auto & spec = selected_specs()[i];
    const auto & row = captures_[i];
    auto entry = node["joints"][spec.key()];
    entry["servo_id"] = static_cast<int>(spec.id);
    entry["direction"] = spec.direction;
    entry["limit_min"] = spec.lower;
    entry["limit_max"] = spec.upper;
    entry["zero"] = row.zero.value_or(-1);
    entry["zero_source"] = row.zero_source;
    entry["raw_min"] = row.raw_min.value_or(-1);
    entry["raw_max"] = row.raw_max.value_or(-1);
    entry["last_sample"] = row.last_sample.value_or(-1);
    entry["range_captured"] = row.range_captured;
    entry["fault"] = row.fault;
  }
  atomic_yaml(file, node, false);
}

void ServoCalibrationSession::restore(const std::filesystem::path & file)
{
  auto node = YAML::LoadFile(file.string());
  if (node["schema"].as<std::string>() != "xlerobot_servo_session/v1" ||
    node["unit_id"].as<std::string>() != unit_id_ ||
    node["bus_identity"].as<std::string>() != bus_identity_ ||
    !node["joints"].IsMap() || node["joints"].size() != selected_specs().size())
  {
    throw std::runtime_error("servo session schema, unit, buses, or joint set differs; use --fresh");
  }
  const auto phase = node["phase"].as<std::string>();
  const std::set<std::string> phases{
    "IDLE", "ZERO_CAPTURED", "RANGE_RECORDING", "PAUSED", "RANGE_CAPTURED", "FINALIZED"};
  if (!phases.count(phase)) {throw std::runtime_error("invalid servo session phase");}
  const auto active = node["active_group"].as<std::string>();
  if (!active.empty()) {group(active);}
  if ((phase == "RANGE_RECORDING" || phase == "PAUSED" || phase == "ZERO_CAPTURED") &&
    active.empty()) {throw std::runtime_error("servo session active group is missing");}
  if ((phase == "FINALIZED" || phase == "RANGE_CAPTURED") && !active.empty()) {
    throw std::runtime_error("completed servo session cannot have an active group");
  }
  auto restored = captures_;
  for (size_t i = 0; i < selected_specs().size(); ++i) {
    const auto & spec = selected_specs()[i];
    const auto entry = node["joints"][spec.key()];
    validate_spec(entry, spec);
    auto & row = restored[i];
    row.zero = optional_raw(entry["zero"]);
    row.zero_source = entry["zero_source"].as<std::string>();
    row.raw_min = optional_raw(entry["raw_min"]);
    row.raw_max = optional_raw(entry["raw_max"]);
    row.last_sample = optional_raw(entry["last_sample"]);
    row.range_captured = entry["range_captured"].as<bool>();
    row.fault = entry["fault"].as<std::string>();
    row.online = false;
    row.position.reset();
    row.read_error.clear();
    const bool source_valid = row.zero_source == "measured" ||
      (row.zero_source.rfind("existing:", 0) == 0 && row.zero_source.size() > 9);
    if ((row.zero && !source_valid) || (!row.zero && !row.zero_source.empty()) ||
      row.raw_min.has_value() != row.raw_max.has_value() ||
      (row.raw_min && (!row.zero || *row.raw_min > *row.raw_max)))
    {
      throw std::runtime_error("invalid servo session capture: " + spec.key());
    }
    if (row.range_captured && (!row.zero || !row.raw_min || !row.raw_max ||
      !row.fault.empty() || *row.zero < *row.raw_min || *row.zero > *row.raw_max ||
      (*row.raw_max - *row.raw_min) < kMinimumCoverage *
      (spec.upper - spec.lower) * kTicksPerRadian))
    {
      throw std::runtime_error("invalid completed servo range: " + spec.key());
    }
  }
  if (phase == "FINALIZED" && std::any_of(restored.begin(), restored.end(),
      [](const auto & row) {return !row.range_captured;}))
  {
    throw std::runtime_error("finalized session is incomplete");
  }
  captures_ = std::move(restored);
  active_group_ = active;
  // A restarted process never resumes sampling or assumes the torque state.
  phase_ = phase == "RANGE_RECORDING" ? "PAUSED" : phase;
}

void ServoCalibrationSession::validate_complete() const
{
  if (recording()) {throw std::runtime_error("finish or pause the active range before finalizing");}
  for (size_t i = 0; i < selected_specs().size(); ++i) {
    const auto & row = captures_[i];
    if (!row.range_captured || !row.zero || !row.raw_min || !row.raw_max || !row.fault.empty() ||
      *row.zero < *row.raw_min || *row.zero > *row.raw_max || coverage(i) < kMinimumCoverage)
    {
      throw std::runtime_error("calibration is incomplete: " + selected_specs()[i].key());
    }
  }
}

void ServoCalibrationSession::finalize(const std::filesystem::path & file)
{
  ensure_mutable();
  validate_complete();
  YAML::Node node;
  node["schema"] = "xlerobot_servo_calibration/v1";
  for (size_t i = 0; i < selected_specs().size(); ++i) {
    const auto & spec = selected_specs()[i];
    const auto & row = captures_[i];
    auto entry = node[spec.group]["joints"][spec.name];
    entry["servo_id"] = static_cast<int>(spec.id);
    entry["direction"] = spec.direction;
    entry["offset"] = *row.zero;
    entry["raw_min"] = *row.raw_min;
    entry["raw_max"] = *row.raw_max;
    entry["limit_min"] = spec.lower;
    entry["limit_max"] = spec.upper;
  }
  if (leader_only_) {
    YAML::Node attachment;
    attachment["schema"] = "xlerobot_servo_calibration/v1";
    attachment["attachment"] = "right_leader";
    attachment["joints"] = node["leader"]["joints"];
    node = attachment;
  }
  atomic_yaml(file, node, true);
  phase_ = "FINALIZED";
  active_group_.clear();
}

void ServoCalibrationSession::recover_finalized_result(const std::filesystem::path & file)
{
  // Covers interruption after publishing result.yaml but before saving FINALIZED
  // to session.yaml. Only adopt a result that exactly matches all completed rows.
  validate_complete();
  auto node = YAML::LoadFile(file.string());
  if (node["schema"].as<std::string>() != "xlerobot_servo_calibration/v1") {
    throw std::runtime_error("existing result has the wrong schema; use --fresh");
  }
  if (leader_only_) {
    if (node["attachment"].as<std::string>("") != "right_leader") {
      throw std::runtime_error("expected a right_leader result");
    }
    YAML::Node wrapped;
    wrapped["leader"]["joints"] = node["joints"];
    node = wrapped;
  }
  for (const auto & name : groups_) {
    if (!node[name]["joints"].IsMap() || node[name]["joints"].size() != group(name).size()) {
      throw std::runtime_error("existing result has the wrong joint set; use --fresh");
    }
  }
  for (size_t i = 0; i < selected_specs().size(); ++i) {
    const auto & spec = selected_specs()[i];
    const auto & row = captures_[i];
    const auto entry = node[spec.group]["joints"][spec.name];
    validate_spec(entry, spec);
    if (entry["offset"].as<int>() != *row.zero || entry["raw_min"].as<int>() != *row.raw_min ||
      entry["raw_max"].as<int>() != *row.raw_max)
    {
      throw std::runtime_error("existing result differs from this session; use --fresh");
    }
  }
  phase_ = "FINALIZED";
  active_group_.clear();
}
}  // namespace xlerobot_feetech
