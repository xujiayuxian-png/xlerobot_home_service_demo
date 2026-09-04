#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "xlerobot_lidar_driver/lidar_types.hpp"

namespace xlerobot_lidar_driver
{

class LidarProtocol
{
public:
  static constexpr size_t kPacketHeaderSize = 10;
  static constexpr size_t kSampleSize = 3;
  static constexpr size_t kSampleDistanceOffset = 1;
  static constexpr double kAngleCorrectionOffsetMm = 21.8;
  static constexpr double kAngleCorrectionBaselineMm = 155.3;

  static uint16_t bytesToUint16(const std::vector<uint8_t> & data, size_t index);
  static size_t expectedPacketSize(uint8_t lsn);
  static uint16_t calculateChecksum(const std::vector<uint8_t> & packet_data);
  static bool isChecksumValid(const std::vector<uint8_t> & packet_data);
  static std::optional<LidarPacket> parsePacket(const std::vector<uint8_t> & packet_data);
};

}  // namespace xlerobot_lidar_driver
