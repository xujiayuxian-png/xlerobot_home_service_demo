#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "xlerobot_lidar_driver/lidar_types.hpp"

namespace xlerobot_lidar_driver
{

class LidarPacketParser
{
public:
  explicit LidarPacketParser(bool checksum_enabled = true);

  void setChecksumEnabled(bool enabled);
  std::optional<LidarPacket> processByte(uint8_t byte);

  uint64_t invalidChecksumCount() const;
  uint16_t lastExpectedChecksum() const;
  uint16_t lastActualChecksum() const;

private:
  enum class State
  {
    kWaitHeader1,
    kWaitHeader2,
    kReadMeta,
    kReadPayload,
  };

  void reset();

  bool checksum_enabled_;
  State state_ = State::kWaitHeader1;
  std::vector<uint8_t> packet_buffer_;
  size_t target_packet_size_ = 0;
  uint64_t invalid_checksum_count_ = 0;
  uint16_t last_expected_checksum_ = 0;
  uint16_t last_actual_checksum_ = 0;
};

}  // namespace xlerobot_lidar_driver
